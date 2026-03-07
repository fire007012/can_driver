#include "can_driver/MtCan.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <iomanip>
#include <iostream>

namespace {
constexpr uint16_t kSendBaseId = 0x140;
constexpr uint16_t kResponseBaseId = 0x240;
constexpr int32_t kDefaultPositionSpeedDps = 100;
constexpr std::size_t kQueriesPerMotorPerCycle = 3;

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
            refreshMotorIds.push_back(static_cast<uint8_t>(id));
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
    uint8_t motorId = static_cast<uint8_t>(Id);
    std::lock_guard<std::mutex> stateLock(stateMutex);
    motorStates[motorId].mode = mode;
    return true;
}

bool MtCan::setVelocity(MotorID Id, int32_t velocity)
{
    if (!canController) {
        return false;
    }
    uint8_t motorId = static_cast<uint8_t>(Id);
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        motorStates[motorId].commandedVelocity = velocity;
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
    return writeAcceleration(static_cast<uint8_t>(id), 0x02, accelDpsPerSec);
}

bool MtCan::setSpeedDeceleration(MotorID id, uint32_t decelDpsPerSec)
{
    return writeAcceleration(static_cast<uint8_t>(id), 0x03, decelDpsPerSec);
}

bool MtCan::setPositionAcceleration(MotorID id, uint32_t accelDpsPerSec)
{
    return writeAcceleration(static_cast<uint8_t>(id), 0x00, accelDpsPerSec);
}

bool MtCan::setPositionDeceleration(MotorID id, uint32_t decelDpsPerSec)
{
    return writeAcceleration(static_cast<uint8_t>(id), 0x01, decelDpsPerSec);
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
    uint8_t motorId = static_cast<uint8_t>(Id);
    int32_t commandedVelocity = 0;
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        MotorState &state = motorStates[motorId];
        state.position = position;
        commandedVelocity = state.commandedVelocity;
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

// [FIX #4] 不再每次 Enable 都设置零点并复位系统
bool MtCan::Enable(MotorID Id)
{
    if (!canController) {
        return false;
    }
    uint8_t motorId = static_cast<uint8_t>(Id);
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        motorStates[motorId].enabled = true;
    }
    // 脉塔协议无独立使能命令。
    // 如需设置零点请单独调用 setZeroPosition()，避免频繁写 ROM。
    return true;
}

// [FIX #3] 使用 0x80 (Motor Off) 而非 0x81 (Stop)
bool MtCan::Disable(MotorID Id)
{
    if (!canController) {
        return false;
    }
    uint8_t motorId = static_cast<uint8_t>(Id);
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
    uint8_t motorId = static_cast<uint8_t>(Id);
    const uint16_t canId = encodeSendCanId(motorId);
    sendFrame(canId, 0x81, {0, 0, 0, 0}); // Motor Stop: 停止运动，保持受控
    return true;
}

// [FIX #5] 返回电机实际位置（从 0x92 多圈角度读回），而非命令值
int64_t MtCan::getPosition(MotorID Id) const
{
    uint8_t motorId = static_cast<uint8_t>(Id);
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
    uint8_t motorId = static_cast<uint8_t>(Id);
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
    uint8_t motorId = static_cast<uint8_t>(Id);
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
    const uint8_t motorId = static_cast<uint8_t>(Id);
    std::lock_guard<std::mutex> stateLock(stateMutex);
    auto it = motorStates.find(motorId);
    return (it != motorStates.end()) ? it->second.enabled : false;
}

bool MtCan::hasFault(MotorID Id) const
{
    const uint8_t motorId = static_cast<uint8_t>(Id);
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
    if (canId < kResponseBaseId || canId >= kResponseBaseId + 0x100) {
        return; // 非本驱动响应帧，静默忽略（避免多设备总线日志洪泛）
    }

    if (frame.dlc == 0) {
        return;
    }

    const uint8_t command = frame.data[0];

    // [FIX #1] 用减法提取电机 ID，而非位掩码
    //   原代码: canId & 0xFF → 0x241 & 0xFF = 0x41 = 65（错误）
    //   修正后: canId - 0x240 → 0x241 - 0x240 = 1（正确）
    const uint8_t nodeId = static_cast<uint8_t>(canId - kResponseBaseId);

    bool shouldResetAfterZero = false;

    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        MotorState &state = motorStates[nodeId];

        switch (command) {

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
