#include "can_driver/MtCan.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>

namespace {
constexpr uint16_t kSendBaseId = 0x140;
constexpr uint16_t kResponseBaseId = 0x240;
constexpr uint16_t kMitSendBaseId = 0x400;
constexpr uint16_t kMitResponseBaseId = 0x500;
constexpr int32_t kDefaultPositionSpeedDps = 100;
constexpr std::size_t kQueriesPerMotorPerCycle = 3;
constexpr double kPi = 3.14159265358979323846;

constexpr double kMitPosMin = -12.5;
constexpr double kMitPosMax = 12.5;
constexpr double kMitVelMin = -45.0;
constexpr double kMitVelMax = 45.0;
constexpr double kMitKpMin = 0.0;
constexpr double kMitKpMax = 500.0;
constexpr double kMitKdMin = 0.0;
constexpr double kMitKdMax = 5.0;
constexpr double kMitTorqueMin = -24.0;
constexpr double kMitTorqueMax = 24.0;
constexpr int32_t kMitPositionRawAbsMax = 71620; // 12.5rad -> 0.01deg
constexpr int32_t kMitAssistErrorRawThreshold = 200; // 2.00deg

int16_t readInt16LE(const CanTransport::Frame &frame, std::size_t index)
{
    if (index + 1 >= frame.dlc) {
        return 0;
    }
    const int value = static_cast<int>(frame.data[index]) |
                      (static_cast<int>(frame.data[index + 1]) << 8);
    return static_cast<int16_t>(value);
}

uint16_t readUInt16LE(const CanTransport::Frame &frame, std::size_t index)
{
    if (index + 1 >= frame.dlc) {
        return 0;
    }
    return static_cast<uint16_t>(static_cast<uint16_t>(frame.data[index]) |
                                 (static_cast<uint16_t>(frame.data[index + 1]) << 8));
}

// 读取 48 位小端有符号整数（用于多圈角度 0x92 应答，DATA[2]~DATA[7]）
int64_t readInt48LE(const CanTransport::Frame &frame, std::size_t index)
{
    if (index + 5 >= frame.dlc) {
        return 0;
    }
    int64_t v = 0;
    for (int i = 5; i >= 0; --i) {
        v = (v << 8) | frame.data[index + static_cast<std::size_t>(i)];
    }
    // 48 位符号扩展
    if (v & (int64_t{1} << 47)) {
        v -= (int64_t{1} << 48);
    }
    return v;
}

uint16_t doubleToUint16(double x, double min, double max)
{
    const double clamped = std::clamp(x, min, max);
    const double ratio = (clamped - min) / (max - min);
    const double scaled = ratio * 65535.0;
    return static_cast<uint16_t>(std::llround(std::clamp(scaled, 0.0, 65535.0)));
}

uint16_t doubleToUint12(double x, double min, double max)
{
    const double clamped = std::clamp(x, min, max);
    const double ratio = (clamped - min) / (max - min);
    const double scaled = ratio * 4095.0;
    return static_cast<uint16_t>(std::llround(std::clamp(scaled, 0.0, 4095.0)));
}

double uint16ToDouble(uint16_t raw, double min, double max)
{
    return (static_cast<double>(raw) / 65535.0) * (max - min) + min;
}

double uint12ToDouble(uint16_t raw, double min, double max)
{
    const uint16_t r = static_cast<uint16_t>(raw & 0x0FFF);
    return (static_cast<double>(r) / 4095.0) * (max - min) + min;
}

// 兼容两种 motorId 写法：
// 1) 节点号：0x01~0xFF
// 2) 完整发送 CAN ID：0x141~0x1FF（常见于现场配置）
uint8_t normalizeMtNodeId(MotorID id)
{
    const uint16_t raw = static_cast<uint16_t>(id);
    if (raw >= 0x141 && raw <= 0x1FF) {
        return static_cast<uint8_t>(raw - 0x140);
    }
    return static_cast<uint8_t>(raw & 0xFF);
}

std::string formatData8(const std::array<uint8_t, 8> &data)
{
    std::ostringstream oss;
    oss << std::hex << std::setfill('0');
    for (std::size_t i = 0; i < 8; ++i) {
        if (i) {
            oss << ' ';
        }
        oss << "0x" << std::setw(2) << static_cast<int>(data[i]);
    }
    return oss.str();
}
} // namespace

std::chrono::milliseconds MtCan::computeRefreshSleep(std::size_t motorCount) const
{
    const double hz = refreshRateHz_.load(std::memory_order_relaxed);
    if (std::isfinite(hz) && hz > 0.0) {
        const auto intervalMs = static_cast<int64_t>(std::llround(1000.0 / hz));
        return std::chrono::milliseconds(std::max<int64_t>(1, intervalMs));
    }
    const std::size_t intervalMs = std::max<std::size_t>(5, motorCount * kQueriesPerMotorPerCycle);
    return std::chrono::milliseconds(intervalMs);
}

MtCan::MtCan(std::shared_ptr<CanTransport> controller)
    : canController(std::move(controller))
{
    loadMitConfigFromEnv();
    if (canController) {
        receiveHandlerId = canController->addReceiveHandler(
            [this](const CanTransport::Frame &frame) { handleResponse(frame); });
    }
}

MtCan::~MtCan()
{
    stopRefreshLoop();
    if (canController && receiveHandlerId != 0) {
        canController->removeReceiveHandler(receiveHandlerId);
    }
}

void MtCan::initializeMotorRefresh(const std::vector<MotorID> &motorIds)
{
    {
        std::lock_guard<std::mutex> lock(refreshMutex);
        refreshMotorIds.clear();
        refreshMotorIds.reserve(motorIds.size());
        for (MotorID id : motorIds) {
            refreshMotorIds.push_back(normalizeMtNodeId(id));
        }
    }

    if (motorIds.empty()) {
        stopRefreshLoop();
        return;
    }

    if (refreshLoopActive.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        broadcastCommunicationTimeout(300);
        return;
    }

    bool expected = false;
    if (!refreshLoopActive.compare_exchange_strong(expected, true)) {
        return;
    }

    refreshThread = std::thread([this]() {
        while (refreshLoopActive.load()) {
            refreshMotorStates();
            std::size_t motorCount = 0;
            {
                std::lock_guard<std::mutex> lock(refreshMutex);
                motorCount = refreshMotorIds.size();
            }
            std::this_thread::sleep_for(this->computeRefreshSleep(motorCount));
        }
    });

    // 电机注册后自动下发通讯中断保护，避免控制链路异常时失控
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    broadcastCommunicationTimeout(300);
}

void MtCan::setRefreshRateHz(double hz)
{
    if (!std::isfinite(hz) || hz <= 0.0) {
        refreshRateHz_.store(0.0, std::memory_order_relaxed);
        return;
    }
    refreshRateHz_.store(hz, std::memory_order_relaxed);
}

bool MtCan::setMode(MotorID Id, MotorMode mode)
{
    uint8_t motorId = normalizeMtNodeId(Id);
    const uint16_t motorIdRaw = static_cast<uint16_t>(Id);
    bool needPrimeMit = false;
    int32_t primePositionRaw = 0;
    uint8_t lastRunMode = 0;

    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        MotorState &state = motorStates[motorId];
        state.mode = mode;
        state.mitModePrimed = false;
        if (mode == MotorMode::Position) {
            // 进入位置模式时清空速度目标，避免 MIT 模式继承旧速度命令导致持续旋转。
            state.commandedVelocity = 0;
            // 多圈角度偶发异常值时，不用于 priming，退回最后位置命令。
            if (std::llabs(state.multiTurnAngle) <= static_cast<long long>(kMitPositionRawAbsMax)) {
                primePositionRaw = static_cast<int32_t>(state.multiTurnAngle);
            } else {
                primePositionRaw = state.position;
            }
            lastRunMode = state.runMode;
            needPrimeMit = mtPositionUseMit_;
        }
    }

    // 主动“预热”MIT模式：释放抱闸 + 请求运行模式 + 发送当前位置保持帧。
    // 目的：避免仅切软件状态但电机侧尚未稳定进入位置/运控状态。
    if (needPrimeMit && canController) {
        const uint16_t canId = encodeSendCanId(motorId);
        sendFrame(canId, 0x77, {0, 0, 0, 0});
        // runmode=0x02 时，先用 A4 当前位置保持强制进入位置环，再发 MIT。
        if (lastRunMode != 0x03) {
            sendA4PositionHoldRaw(motorId, primePositionRaw, 600);
        }
        requestRunMode(motorId);
        (void)sendMitPositionCommand(motorIdRaw, motorId, primePositionRaw);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        if (lastRunMode != 0x03) {
            sendA4PositionHoldRaw(motorId, primePositionRaw, 600);
        }
        (void)sendMitPositionCommand(motorIdRaw, motorId, primePositionRaw);
        requestRunMode(motorId);
    }

    return true;
}

bool MtCan::setVelocity(MotorID Id, int32_t velocity)
{
    if (!canController) {
        return false;
    }
    uint8_t motorId = normalizeMtNodeId(Id);
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        MotorState &state = motorStates[motorId];
        state.commandedVelocity = velocity;
        state.mode = MotorMode::Velocity;
    }
    const uint16_t canId = encodeSendCanId(motorId);

    std::array<uint8_t, 4> payload{
        static_cast<uint8_t>(velocity & 0xFF),
        static_cast<uint8_t>((velocity >> 8) & 0xFF),
        static_cast<uint8_t>((velocity >> 16) & 0xFF),
        static_cast<uint8_t>((velocity >> 24) & 0xFF)};
    sendFrame(canId, 0xA2, payload);
    return true;
}

bool MtCan::setAcceleration(MotorID Id, int32_t acceleration)
{
    if (acceleration <= 0) {
        acceleration = 100;
    }
    return setSpeedAcceleration(Id, static_cast<uint32_t>(acceleration));
}

bool MtCan::setDeceleration(MotorID Id, int32_t deceleration)
{
    if (deceleration <= 0) {
        deceleration = 100;
    }
    return setSpeedDeceleration(Id, static_cast<uint32_t>(deceleration));
}

bool MtCan::setCommunicationTimeout(uint32_t timeoutMs)
{
    if (!canController) {
        return false;
    }

    std::vector<uint8_t> motorIds;
    {
        std::lock_guard<std::mutex> lock(refreshMutex);
        motorIds = refreshMotorIds;
    }
    if (motorIds.empty()) {
        std::cerr << "[MtCan] setCommunicationTimeout: no motors registered\n";
        return false;
    }

    for (uint8_t motorId : motorIds) {
        const uint16_t canId = encodeSendCanId(motorId);
        CanTransport::Frame frame;
        frame.id = canId;
        frame.dlc = 8;
        frame.isExtended = false;
        frame.isRemoteRequest = false;
        frame.data.fill(0);
        frame.data[0] = 0xB3;
        frame.data[4] = static_cast<uint8_t>(timeoutMs & 0xFF);
        frame.data[5] = static_cast<uint8_t>((timeoutMs >> 8) & 0xFF);
        frame.data[6] = static_cast<uint8_t>((timeoutMs >> 16) & 0xFF);
        frame.data[7] = static_cast<uint8_t>((timeoutMs >> 24) & 0xFF);
        canController->send(frame);
    }

    std::cout << "[MtCan] Communication timeout set to " << timeoutMs
              << " ms for " << motorIds.size() << " motor(s)\n";
    return true;
}

bool MtCan::writeAcceleration(uint8_t motorId, uint8_t index, uint32_t value)
{
    if (!canController) {
        return false;
    }
    value = std::max<uint32_t>(100, std::min<uint32_t>(60000, value));

    const uint16_t canId = encodeSendCanId(motorId);
    CanTransport::Frame frame;
    frame.id = canId;
    frame.dlc = 8;
    frame.isExtended = false;
    frame.isRemoteRequest = false;
    frame.data.fill(0);
    frame.data[0] = 0x43;
    frame.data[1] = index;
    frame.data[4] = static_cast<uint8_t>(value & 0xFF);
    frame.data[5] = static_cast<uint8_t>((value >> 8) & 0xFF);
    frame.data[6] = static_cast<uint8_t>((value >> 16) & 0xFF);
    frame.data[7] = static_cast<uint8_t>((value >> 24) & 0xFF);
    canController->send(frame);
    return true;
}

bool MtCan::setSpeedAcceleration(MotorID id, uint32_t accelDpsPerSec)
{
    return writeAcceleration(normalizeMtNodeId(id), 0x02, accelDpsPerSec);
}

bool MtCan::setSpeedDeceleration(MotorID id, uint32_t decelDpsPerSec)
{
    return writeAcceleration(normalizeMtNodeId(id), 0x03, decelDpsPerSec);
}

bool MtCan::setPositionAcceleration(MotorID id, uint32_t accelDpsPerSec)
{
    return writeAcceleration(normalizeMtNodeId(id), 0x00, accelDpsPerSec);
}

bool MtCan::setPositionDeceleration(MotorID id, uint32_t decelDpsPerSec)
{
    return writeAcceleration(normalizeMtNodeId(id), 0x01, decelDpsPerSec);
}

void MtCan::broadcastCommunicationTimeout(uint32_t timeoutMs)
{
    (void)setCommunicationTimeout(timeoutMs);
}

bool MtCan::setPosition(MotorID Id, int32_t position)
{
    if (!canController) {
        return false;
    }
    const uint16_t motorIdRaw = static_cast<uint16_t>(Id);
    uint8_t motorId = normalizeMtNodeId(Id);
    int32_t commandedVelocity = 0;
    bool needPrimeMit = false;
    int32_t primePositionRaw = position;
    uint8_t lastRunMode = 0;
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        MotorState &state = motorStates[motorId];
        state.position = position;
        state.mode = MotorMode::Position;
        // 位置控制默认不带速度目标，防止持续旋转。
        state.commandedVelocity = 0;
        if (mtPositionUseMit_ && !state.mitModePrimed) {
            needPrimeMit = true;
            if (std::llabs(state.multiTurnAngle) <= static_cast<long long>(kMitPositionRawAbsMax)) {
                primePositionRaw = static_cast<int32_t>(state.multiTurnAngle);
            } else {
                primePositionRaw = position;
            }
            lastRunMode = state.runMode;
        }
        commandedVelocity = state.commandedVelocity;
    }

    if (mtPositionUseMit_) {
        if (needPrimeMit) {
            const uint16_t canId = encodeSendCanId(motorId);
            sendFrame(canId, 0x77, {0, 0, 0, 0});
            if (lastRunMode != 0x03) {
                sendA4PositionHoldRaw(motorId, primePositionRaw, 600);
            }
            requestRunMode(motorId);
            (void)sendMitPositionCommand(motorIdRaw, motorId, primePositionRaw);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            if (lastRunMode != 0x03) {
                sendA4PositionHoldRaw(motorId, primePositionRaw, 600);
            }
            requestRunMode(motorId);
        }
        return sendMitPositionCommand(motorIdRaw, motorId, position);
    }

    const uint16_t canId = encodeSendCanId(motorId);

    // 0xA4 DATA[2-3] = maxSpeed (uint16_t, 1 dps/LSB)
    // commandedVelocity 来自 0xA2 语义（0.01 dps/LSB），发送前需换算成 dps。
    std::int64_t absVel001Dps = std::llabs(static_cast<long long>(commandedVelocity));
    std::int64_t absVelDps = 0;
    if (absVel001Dps == 0) {
        absVelDps = kDefaultPositionSpeedDps;
    } else {
        absVelDps = (absVel001Dps + 50) / 100; // 四舍五入到 1 dps/LSB
        if (absVelDps == 0) {
            absVelDps = 1;
        }
    }
    const uint16_t maxSpeed = static_cast<uint16_t>(std::min<std::int64_t>(absVelDps, 0xFFFF));

    CanTransport::Frame frame;
    frame.id = canId;
    frame.dlc = 8;
    frame.isExtended = false;
    frame.isRemoteRequest = false;
    frame.data[0] = 0xA4;
    frame.data[1] = 0x00;
    frame.data[2] = static_cast<uint8_t>(maxSpeed & 0xFF);
    frame.data[3] = static_cast<uint8_t>((maxSpeed >> 8) & 0xFF);
    frame.data[4] = static_cast<uint8_t>(position & 0xFF);
    frame.data[5] = static_cast<uint8_t>((position >> 8) & 0xFF);
    frame.data[6] = static_cast<uint8_t>((position >> 16) & 0xFF);
    frame.data[7] = static_cast<uint8_t>((position >> 24) & 0xFF);

    canController->send(frame);
    return true;
}

bool MtCan::sendMitPositionCommand(uint16_t motorIdRaw, uint8_t nodeId, int32_t positionRaw)
{
    const int32_t positionRawSafe =
        std::clamp(positionRaw, -kMitPositionRawAbsMax, kMitPositionRawAbsMax);

    // 兼容现有 MT 位置命令输入语义：positionRaw 单位为 0.01°。
    // MIT 期望位置单位为 rad，因此这里做 0.01° -> rad 转换。
    const double positionRad = static_cast<double>(positionRawSafe) * (kPi / 18000.0);

    MitCommand cmd;
    cmd.positionRad = positionRad;
    // 位置模式默认 v_des = 0，确保按位置收敛，不继承历史速度命令。
    cmd.velocityRadPerSec = 0.0;
    cmd.kp = mitDefaultKp_;
    cmd.kd = mitDefaultKd_;
    cmd.torqueNm = mitDefaultTorqueNm_;

    bool primed = false;
    uint8_t runmode = 0;
    int32_t currentRaw = 0;
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        const auto it = motorStates.find(nodeId);
        if (it != motorStates.end()) {
            primed = it->second.mitModePrimed;
            runmode = it->second.runMode;
            currentRaw = static_cast<int32_t>(std::clamp<int64_t>(
                it->second.multiTurnAngle,
                std::numeric_limits<int32_t>::min(),
                std::numeric_limits<int32_t>::max()));
        }
    }

    // 兼容两类设备寻址：
    // 1) 文档常见：0x400 + 节点号（节点号=0x141-0x140）
    // 2) 现场部分固件：0x400 + 完整发送ID（例如 0x141 -> 0x541）
    const uint16_t mitByNode = static_cast<uint16_t>(kMitSendBaseId + nodeId);
    if (mtDebugMit_) {
        std::cout << "[MtCan][MIT][TX-PLAN] raw_motor_id=0x" << std::hex << motorIdRaw
                  << " node_id=0x" << static_cast<int>(nodeId)
                  << " mit_id(node)=0x" << mitByNode
                  << std::dec
                  << " p_raw_0.01deg=" << positionRaw
                  << " p_raw_safe_0.01deg=" << positionRawSafe
                  << " p_rad=" << positionRad
                  << " v_rad_s=" << cmd.velocityRadPerSec
                  << " kp=" << cmd.kp
                  << " kd=" << cmd.kd
                  << " t_ff=" << cmd.torqueNm
                  << " primed=" << (primed ? 1 : 0)
                  << " runmode=0x" << std::hex << static_cast<int>(runmode) << std::dec
                  << '\n';
    }
    sendMitFrame(mitByNode, cmd);

    // 位置环已建立但仍存在明显位置误差时，追加 A4 目标辅助，避免“首帧动、后续不动”。
    // A4 与 MIT 并行下发，仅作为驱动侧位置环保持辅助。
    const int32_t errRaw = positionRawSafe - currentRaw;
    if (runmode == 0x03 && std::llabs(static_cast<long long>(errRaw)) >= kMitAssistErrorRawThreshold) {
        sendA4PositionHoldRaw(nodeId, positionRawSafe, 900);
        if (mtDebugMit_) {
            std::cout << "[MtCan][MODE-ASSIST] A4 assist node=0x" << std::hex
                      << static_cast<int>(nodeId) << std::dec
                      << " target_raw_0.01deg=" << positionRawSafe
                      << " current_raw_0.01deg=" << currentRaw
                      << " err_raw_0.01deg=" << errRaw
                      << " max_speed_dps=900\n";
        }
    }

    if (motorIdRaw >= 0x141 && motorIdRaw <= 0x1FF) {
        const uint16_t mitByFullId = static_cast<uint16_t>(kMitSendBaseId + motorIdRaw);
        if (mitByFullId != mitByNode) {
            if (mtDebugMit_) {
                std::cout << "[MtCan][MIT][TX-PLAN] mit_id(full)=0x" << std::hex << mitByFullId
                          << std::dec << " (compat)" << '\n';
            }
            sendMitFrame(mitByFullId, cmd);
        }
    }

    return true;
}

void MtCan::sendMitFrame(uint16_t mitCanId, const MitCommand &cmd) const
{
    if (!canController) {
        return;
    }

    const uint16_t p_u16 = doubleToUint16(cmd.positionRad, kMitPosMin, kMitPosMax);
    const uint16_t v_u12 = doubleToUint12(cmd.velocityRadPerSec, kMitVelMin, kMitVelMax);
    const uint16_t kp_u12 = doubleToUint12(cmd.kp, kMitKpMin, kMitKpMax);
    const uint16_t kd_u12 = doubleToUint12(cmd.kd, kMitKdMin, kMitKdMax);
    const uint16_t t_u12 = doubleToUint12(cmd.torqueNm, kMitTorqueMin, kMitTorqueMax);

    CanTransport::Frame frame;
    frame.id = static_cast<uint32_t>(mitCanId);
    frame.dlc = 8;
    frame.isExtended = false;
    frame.isRemoteRequest = false;
    frame.data[0] = static_cast<uint8_t>((p_u16 >> 8) & 0xFF);
    frame.data[1] = static_cast<uint8_t>(p_u16 & 0xFF);
    frame.data[2] = static_cast<uint8_t>((v_u12 >> 4) & 0xFF);
    frame.data[3] = static_cast<uint8_t>(((v_u12 & 0x0F) << 4) | ((kp_u12 >> 8) & 0x0F));
    frame.data[4] = static_cast<uint8_t>(kp_u12 & 0xFF);
    frame.data[5] = static_cast<uint8_t>((kd_u12 >> 4) & 0xFF);
    frame.data[6] = static_cast<uint8_t>(((kd_u12 & 0x0F) << 4) | ((t_u12 >> 8) & 0x0F));
    frame.data[7] = static_cast<uint8_t>(t_u12 & 0xFF);

    if (mtDebugMit_) {
        std::cout << "[MtCan][MIT][TX] can_id=0x" << std::hex << mitCanId
                  << " dlc=8 data=" << formatData8(frame.data) << std::dec << '\n';
    }
    canController->send(frame);
}

void MtCan::loadMitConfigFromEnv()
{
    if (const char *v = std::getenv("CAN_DRIVER_MT_USE_MIT_POSITION")) {
        const std::string s(v);
        mtPositionUseMit_ = !(s == "0" || s == "false" || s == "FALSE");
    }

    if (const char *v = std::getenv("CAN_DRIVER_MT_MIT_DEFAULT_KP")) {
        mitDefaultKp_ = std::clamp(std::atof(v), kMitKpMin, kMitKpMax);
    }
    if (const char *v = std::getenv("CAN_DRIVER_MT_MIT_DEFAULT_KD")) {
        mitDefaultKd_ = std::clamp(std::atof(v), kMitKdMin, kMitKdMax);
    }
    if (const char *v = std::getenv("CAN_DRIVER_MT_MIT_DEFAULT_TORQUE")) {
        mitDefaultTorqueNm_ = std::clamp(std::atof(v), kMitTorqueMin, kMitTorqueMax);
    }
    if (const char *v = std::getenv("CAN_DRIVER_MT_DEBUG_MIT")) {
        const std::string s(v);
        mtDebugMit_ = !(s == "0" || s == "false" || s == "FALSE");
    }

    std::cout << "[MtCan] MIT position mode " << (mtPositionUseMit_ ? "enabled" : "disabled")
              << ", kp=" << mitDefaultKp_
              << ", kd=" << mitDefaultKd_
              << ", t_ff=" << mitDefaultTorqueNm_
              << ", debug=" << (mtDebugMit_ ? "on" : "off") << "\n";
}

// [FIX #4] 不再每次 Enable 都设置零点并复位系统
bool MtCan::Enable(MotorID Id)
{
    if (!canController) {
        return false;
    }
    uint8_t motorId = normalizeMtNodeId(Id);
    const uint16_t canId = encodeSendCanId(motorId);
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        motorStates[motorId].enabled = true;
    }

    // 显式唤醒流程：
    // 1) 释放抱闸(0x77，若设备无抱闸则通常忽略)
    // 2) 下发停止(0x81)建立受控状态
    // 3) 主动触发状态查询，便于快速观察在线状态
    sendFrame(canId, 0x77, {0, 0, 0, 0});
    sendFrame(canId, 0x81, {0, 0, 0, 0});
    requestState(motorId);
    requestError(motorId);
    requestMultiTurnAngle(motorId);

    return true;
}

// [FIX #3] 使用 0x80 (Motor Off) 而非 0x81 (Stop)
bool MtCan::Disable(MotorID Id)
{
    if (!canController) {
        return false;
    }
    uint8_t motorId = normalizeMtNodeId(Id);
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        motorStates[motorId].enabled = false;
    }
    const uint16_t canId = encodeSendCanId(motorId);
    sendFrame(canId, 0x80, {0, 0, 0, 0}); // Motor Off: 关闭输出，清除运行状态
    return true;
}

bool MtCan::Stop(MotorID Id)
{
    if (!canController) {
        return false;
    }
    uint8_t motorId = normalizeMtNodeId(Id);
    const uint16_t canId = encodeSendCanId(motorId);
    sendFrame(canId, 0x81, {0, 0, 0, 0}); // Motor Stop: 停止运动，保持受控
    return true;
}

// [FIX #5] 返回电机实际位置（从 0x92 多圈角度读回），而非命令值
int64_t MtCan::getPosition(MotorID Id) const
{
    uint8_t motorId = normalizeMtNodeId(Id);
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        auto it = motorStates.find(motorId);
        if (it != motorStates.end()) {
            return it->second.multiTurnAngle;
        }
    }
    requestMultiTurnAngle(motorId);
    return 0;
}

int16_t MtCan::getCurrent(MotorID Id) const
{
    uint8_t motorId = normalizeMtNodeId(Id);
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        auto it = motorStates.find(motorId);
        if (it != motorStates.end()) {
            return static_cast<int16_t>(std::lround(it->second.current * 100));
        }
    }
    requestState(motorId);
    return 0;
}

// [FIX #7] 移除 velocity == 0 的不可靠刷新判断
int16_t MtCan::getVelocity(MotorID Id) const
{
    uint8_t motorId = normalizeMtNodeId(Id);
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        auto it = motorStates.find(motorId);
        if (it != motorStates.end()) {
            return it->second.velocity;
        }
    }
    requestState(motorId);
    return 0;
}

bool MtCan::isEnabled(MotorID Id) const
{
    const uint8_t motorId = normalizeMtNodeId(Id);
    std::lock_guard<std::mutex> stateLock(stateMutex);
    auto it = motorStates.find(motorId);
    return (it != motorStates.end()) ? it->second.enabled : false;
}

bool MtCan::hasFault(MotorID Id) const
{
    const uint8_t motorId = normalizeMtNodeId(Id);
    std::lock_guard<std::mutex> stateLock(stateMutex);
    auto it = motorStates.find(motorId);
    return (it != motorStates.end()) ? it->second.error : false;
}

uint16_t MtCan::encodeSendCanId(uint8_t motorId) const
{
    return static_cast<uint16_t>(kSendBaseId + motorId);
}

void MtCan::sendFrame(uint16_t canId, uint8_t command, const std::array<uint8_t, 4> &payload) const
{
    if (!canController) {
        std::cerr << "[MtCan] CAN controller not initialized\n";
        return;
    }

    CanTransport::Frame frame;
    frame.id = canId;
    frame.dlc = 8;
    frame.isExtended = false;
    frame.isRemoteRequest = false;
    frame.data[0] = command;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = payload[0];
    frame.data[5] = payload[1];
    frame.data[6] = payload[2];
    frame.data[7] = payload[3];
    canController->send(frame);
}

// [FIX #2] DLC 改为 8，数据全部清零
void MtCan::requestState(uint8_t motorId) const
{
    if (!canController) {
        return;
    }
    const uint16_t canId = encodeSendCanId(motorId);

    CanTransport::Frame frame;
    frame.id = canId;
    frame.dlc = 8;
    frame.isExtended = false;
    frame.isRemoteRequest = false;
    frame.data.fill(0);
    frame.data[0] = 0x9C;
    canController->send(frame);
}

// [FIX #2] DLC 改为 8，数据全部清零
void MtCan::requestError(uint8_t motorId) const
{
    if (!canController) {
        return;
    }
    const uint16_t canId = encodeSendCanId(motorId);

    CanTransport::Frame frame;
    frame.id = canId;
    frame.dlc = 8;
    frame.isExtended = false;
    frame.isRemoteRequest = false;
    frame.data.fill(0);
    frame.data[0] = 0x9A;
    canController->send(frame);
}

void MtCan::requestRunMode(uint8_t motorId) const
{
    if (!canController) {
        return;
    }
    const uint16_t canId = encodeSendCanId(motorId);

    CanTransport::Frame frame;
    frame.id = canId;
    frame.dlc = 8;
    frame.isExtended = false;
    frame.isRemoteRequest = false;
    frame.data.fill(0);
    frame.data[0] = 0x70;
    canController->send(frame);
}

// [FIX #5 NEW] 请求多圈角度 (0x92) 以获取实际位置
void MtCan::requestMultiTurnAngle(uint8_t motorId) const
{
    if (!canController) {
        return;
    }
    const uint16_t canId = encodeSendCanId(motorId);

    CanTransport::Frame frame;
    frame.id = canId;
    frame.dlc = 8;
    frame.isExtended = false;
    frame.isRemoteRequest = false;
    frame.data.fill(0);
    frame.data[0] = 0x92;
    canController->send(frame);
}

void MtCan::sendA4PositionHoldRaw(uint8_t motorId, int32_t positionRaw, uint16_t maxSpeedDps) const
{
    if (!canController) {
        return;
    }
    const uint16_t canId = encodeSendCanId(motorId);
    CanTransport::Frame frame;
    frame.id = canId;
    frame.dlc = 8;
    frame.isExtended = false;
    frame.isRemoteRequest = false;
    frame.data[0] = 0xA4;
    frame.data[1] = 0x00;
    frame.data[2] = static_cast<uint8_t>(maxSpeedDps & 0xFF);
    frame.data[3] = static_cast<uint8_t>((maxSpeedDps >> 8) & 0xFF);
    frame.data[4] = static_cast<uint8_t>(positionRaw & 0xFF);
    frame.data[5] = static_cast<uint8_t>((positionRaw >> 8) & 0xFF);
    frame.data[6] = static_cast<uint8_t>((positionRaw >> 16) & 0xFF);
    frame.data[7] = static_cast<uint8_t>((positionRaw >> 24) & 0xFF);
    canController->send(frame);

    if (mtDebugMit_) {
        std::cout << "[MtCan][MODE-PRIME] A4 hold sent node=0x" << std::hex << static_cast<int>(motorId)
                  << std::dec << " pos_raw_0.01deg=" << positionRaw
                  << " max_speed_dps=" << maxSpeedDps << "\n";
    }
}

void MtCan::resetSystem(uint8_t motorId) const
{
    const uint16_t canId = encodeSendCanId(motorId);
    sendFrame(canId, 0x76, {0, 0, 0, 0});
}

void MtCan::setZeroPosition(uint8_t motorId) const
{
    const uint16_t canId = encodeSendCanId(motorId);
    sendFrame(canId, 0x64, {0, 0, 0, 0});
}

// [FIX #5] 增加多圈角度轮询
void MtCan::refreshMotorStates()
{
    std::vector<uint8_t> motorIds;
    {
        std::lock_guard<std::mutex> lock(refreshMutex);
        motorIds = refreshMotorIds;
    }

    if (!canController) {
        return;
    }

    for (uint8_t motorId : motorIds) {
        if (!refreshLoopActive.load()) {
            break;
        }
        requestState(motorId);            // 0x9C: 温度、电流、速度、编码器
        requestMultiTurnAngle(motorId);   // 0x92: 多圈角度（实际位置）
        requestError(motorId);            // 0x9A: 错误标志
        requestRunMode(motorId);          // 0x70: 当前运行模式
    }
}

void MtCan::stopRefreshLoop()
{
    refreshLoopActive.store(false);

    if (refreshThread.joinable()) {
        refreshThread.join();
    }
}

// [FIX #1] 重写 handleResponse，修正 nodeId 提取
void MtCan::handleResponse(const CanTransport::Frame &frame)
{
    if (frame.isExtended) {
        return;
    }

    const uint16_t canId = static_cast<uint16_t>(frame.id & 0x7FF);
    const bool isClassicResponse = (canId >= kResponseBaseId && canId < kResponseBaseId + 0x100);
    const bool isMitResponse = (canId >= kMitResponseBaseId && canId < kMitResponseBaseId + 0x100);
    if (!isClassicResponse && !isMitResponse) {
        return; // 非本驱动响应帧，静默忽略（避免多设备总线日志洪泛）
    }

    if (frame.dlc == 0) {
        return;
    }

    const uint8_t command = frame.data[0];

    // [FIX #1] 用减法提取电机 ID，而非位掩码
    //   原代码: canId & 0xFF → 0x241 & 0xFF = 0x41 = 65（错误）
    //   修正后: canId - 0x240 → 0x241 - 0x240 = 1（正确）
    const uint8_t nodeId = isMitResponse
                               ? static_cast<uint8_t>(canId - kMitResponseBaseId)
                               : static_cast<uint8_t>(canId - kResponseBaseId);

    bool shouldResetAfterZero = false;

    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        MotorState &state = motorStates[nodeId];

        if (isMitResponse && frame.dlc >= 6) {
            // MIT 回包: [0]=id, [1..2]=p(16), [3..4高4位]=v(12), [4低4位..5]=t(12)
            const uint16_t p_u16 =
                static_cast<uint16_t>((static_cast<uint16_t>(frame.data[1]) << 8) |
                                      static_cast<uint16_t>(frame.data[2]));
            const uint16_t v_u12 =
                static_cast<uint16_t>((static_cast<uint16_t>(frame.data[3]) << 4) |
                                      ((static_cast<uint16_t>(frame.data[4]) >> 4) & 0x0F));
            const uint16_t t_u12 =
                static_cast<uint16_t>(((static_cast<uint16_t>(frame.data[4]) & 0x0F) << 8) |
                                      static_cast<uint16_t>(frame.data[5]));

            const double p_rad = uint16ToDouble(p_u16, kMitPosMin, kMitPosMax);
            const double v_rad_s = uint12ToDouble(v_u12, kMitVelMin, kMitVelMax);
            const double t_nm = uint12ToDouble(t_u12, kMitTorqueMin, kMitTorqueMax);

            if (mtDebugMit_) {
                std::cout << "[MtCan][MIT][RX] can_id=0x" << std::hex << canId
                          << " node=0x" << static_cast<int>(nodeId)
                          << " data=" << formatData8(frame.data)
                          << std::dec
                          << " p_rad=" << p_rad
                          << " v_rad_s=" << v_rad_s
                          << " t_nm=" << t_nm << '\n';
            }

            state.multiTurnAngle = static_cast<int64_t>(std::llround(p_rad * (18000.0 / kPi)));
            state.velocity = static_cast<int16_t>(
                std::llround(v_rad_s * (180.0 / kPi)));
            state.current = t_nm;
            state.mode = MotorMode::Position;
            return;
        }

        switch (command) {

        // ── 运行模式读取应答 (0x70) ───────────
        case 0x70: {
            if (frame.dlc >= 8) {
                const uint8_t runmode = frame.data[7];
                state.runMode = runmode;
                if (runmode == 0x03) {
                    state.mode = MotorMode::Position;
                    state.mitModePrimed = true;
                } else if (runmode == 0x02) {
                    state.mode = MotorMode::Velocity;
                    state.mitModePrimed = false;
                }
                if (mtDebugMit_) {
                    std::cout << "[MtCan][MODE] node=0x" << std::hex << static_cast<int>(nodeId)
                              << " runmode=0x" << static_cast<int>(runmode)
                              << std::dec
                              << " (0x01=current,0x02=velocity,0x03=position)\n";
                }
            }
            break;
        }

        // ── 读取电机状态2应答 (0x9C) ──────────
        case 0x9C: {
            // [FIX #6] 完整解析: 温度、电流、速度、编码器位置
            if (frame.dlc >= 8) {
                state.temperature = static_cast<int8_t>(frame.data[1]);
                const int16_t rawCurrent = readInt16LE(frame, 2);
                state.current = static_cast<double>(rawCurrent) / 100.0;
                // 速度 int16_t, 单位 1 dps/LSB（保持协议原始单位）
                state.velocity = readInt16LE(frame, 4);
                state.encoderPosition = readUInt16LE(frame, 6);
            }
            break;
        }

        // ── 读取电机状态1和错误标志应答 (0x9A) ──
        case 0x9A: {
            if (frame.dlc >= 8) {
                state.temperature = static_cast<int8_t>(frame.data[1]);
                state.voltageRaw1 = readUInt16LE(frame, 2);
                state.voltageRaw2 = readUInt16LE(frame, 4);
                const uint16_t errorCode = readUInt16LE(frame, 6);
                state.error = errorCode != 0;
                if (state.error) {
                    std::cerr << "[MtCan] Motor " << static_cast<int>(nodeId)
                              << " error code 0x" << std::hex << errorCode
                              << std::dec << '\n';
                }
            }
            break;
        }

        // ── 多圈角度应答 (0x92) ────────────────
        case 0x92: {
            // DATA[1] = NULL, DATA[2~7] = int48_t LE, 单位 0.01°/LSB
            if (frame.dlc >= 8) {
                state.multiTurnAngle = readInt48LE(frame, 2);
            }
            break;
        }

        // ── 运动命令及开关命令应答（共用状态格式）──
        case 0xA1: // 转矩闭环
        case 0xA2: // 速度闭环
        case 0xA4: // 绝对位置
        case 0xA6: // 单圈位置
        case 0xA8: // 增量位置
        case 0xA9: // 力位混合控制
        case 0x80: // Motor Off
        case 0x81: // Motor Stop
        {
            // 应答格式与 0x9C 一致: temp(1), iq(2), speed(2), encoder(2)
            if (frame.dlc >= 8) {
                state.temperature = static_cast<int8_t>(frame.data[1]);
                const int16_t rawCurrent = readInt16LE(frame, 2);
                state.current = static_cast<double>(rawCurrent) / 100.0;
                state.velocity = readInt16LE(frame, 4);
                state.encoderPosition = readUInt16LE(frame, 6);
            }
            break;
        }

        // ── 设置零点应答 (0x64) ──────────────
        case 0x64:
            shouldResetAfterZero = true;
            break;

        // ── 通讯中断保护设置应答 (0xB3) ────────
        case 0xB3: {
            if (frame.dlc >= 8) {
                const uint32_t confirmedTimeout =
                    static_cast<uint32_t>(frame.data[4]) |
                    (static_cast<uint32_t>(frame.data[5]) << 8) |
                    (static_cast<uint32_t>(frame.data[6]) << 16) |
                    (static_cast<uint32_t>(frame.data[7]) << 24);
                std::cout << "[MtCan] Motor " << static_cast<int>(nodeId)
                          << " communication timeout confirmed: "
                          << confirmedTimeout << " ms\n";
            }
            break;
        }

        // ── 加减速度设置应答 (0x43) ──────────
        case 0x43: {
            if (frame.dlc >= 8) {
                const uint8_t accelIndex = frame.data[1];
                const uint32_t confirmedAccel =
                    static_cast<uint32_t>(frame.data[4]) |
                    (static_cast<uint32_t>(frame.data[5]) << 8) |
                    (static_cast<uint32_t>(frame.data[6]) << 16) |
                    (static_cast<uint32_t>(frame.data[7]) << 24);
                const char *names[] = {"pos_accel", "pos_decel", "spd_accel", "spd_decel"};
                const char *name = (accelIndex <= 3) ? names[accelIndex] : "unknown";
                std::cout << "[MtCan] Motor " << static_cast<int>(nodeId)
                          << " " << name << " confirmed: "
                          << confirmedAccel << " dps/s\n";
            }
            break;
        }

        default:
            break;
        }
    }

    // 设置零点后需要系统复位才能生效（在锁外调用，避免死锁）
    if (shouldResetAfterZero) {
        resetSystem(nodeId);
    }
}
