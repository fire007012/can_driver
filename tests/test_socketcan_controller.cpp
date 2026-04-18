#include <gtest/gtest.h>
#include <can_driver/SocketCanController.h>
#include <linux/can.h>

// 通过友元访问私有转换函数，直接验证协议层与 socketcan 帧的互转。
class SocketCanControllerTestAccessor {
public:
  static can_frame toSock(SocketCanController& c, const CanTransport::Frame& f){ return c.toLinuxCanFrame(f); }
  static CanTransport::Frame fromSock(SocketCanController& c, const can_frame& f){ return c.fromLinuxCanFrame(f); }
  static void dispatch(SocketCanController& c, const CanTransport::Frame& f){ c.dispatchReceive(f); }
};

TEST(SocketCanController, EncodeDecodeRoundtripAndBounds){
  // dlc=10 时应被截断到 8，且 encode/decode 内容保持一致。
  SocketCanController ctrl;
  CanTransport::Frame f{}; f.id=0x123; f.isExtended=true; f.isRemoteRequest=false; f.dlc=10; // >8
  for(size_t i=0;i<f.data.size();++i) f.data[i]=static_cast<uint8_t>(i+1);

  auto sf = SocketCanControllerTestAccessor::toSock(ctrl,f);
  EXPECT_EQ((sf.can_id & CAN_EFF_MASK),f.id);
  EXPECT_NE((sf.can_id & CAN_EFF_FLAG),0u);
  EXPECT_EQ((sf.can_id & CAN_RTR_FLAG),0u);
  EXPECT_LE(sf.can_dlc,8);
  for(size_t i=0;i<sf.can_dlc;++i) EXPECT_EQ(sf.data[i], f.data[i]);

  auto back = SocketCanControllerTestAccessor::fromSock(ctrl,sf);
  EXPECT_EQ(back.id,f.id);
  EXPECT_TRUE(back.isExtended);
  EXPECT_FALSE(back.isRemoteRequest);
  EXPECT_EQ(back.dlc,sf.can_dlc);
  for(size_t i=0;i<back.dlc;++i) EXPECT_EQ(back.data[i], sf.data[i]);
}

TEST(SocketCanController, HandlerLifecycleAndDispatch){
  // 验证注册、分发、移除和 no-op 分支。
  SocketCanController ctrl;
  size_t called=0;
  auto id = ctrl.addReceiveHandler([&](const CanTransport::Frame&){ ++called; });
  EXPECT_NE(id,0u);

  CanTransport::Frame f{}; f.id=1; f.dlc=1; f.data[0]=0xAA;
  SocketCanControllerTestAccessor::dispatch(ctrl,f);
  EXPECT_EQ(called,1u);

  ctrl.removeReceiveHandler(id);
  SocketCanControllerTestAccessor::dispatch(ctrl,f);
  EXPECT_EQ(called,1u);

  ctrl.removeReceiveHandler(0); // no-op
}

TEST(SocketCanController, ShutdownResetsState){
  // shutdown 后 handler ID 重新从 1 开始，说明内部状态已清空。
  SocketCanController ctrl;
  auto id1 = ctrl.addReceiveHandler([](auto const&){});
  ctrl.removeReceiveHandler(id1);
  ctrl.shutdown();
  auto id2 = ctrl.addReceiveHandler([](auto const&){});
  EXPECT_EQ(id2,1u);
}
