#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <nao_camera/base_v4l2_device_interface.hpp>
#include <nao_camera/nao_camera_device.hpp>
#include <nao_camera/exception.hpp>

using nao_camera::BaseV4l2DeviceInterface;
using nao_camera::NaoCameraDevice;
using nao_camera::NaoCameraDeviceError;
using nao_camera::PollResult;
using testing::NiceMock;

constexpr NaoCameraDevice::Settings DEFAULT_SETTINGS = {320, 240, false, 30};

class MockV4l2DeviceInterface : public BaseV4l2DeviceInterface {
public:
  // NOLINTBEGIN(misc-non-private-member-variables-in-classes)
  MOCK_METHOD(int, setImageFormat, (uint32_t, uint32_t), (override));
  MOCK_METHOD(int, setStreamParameters, (uint32_t), (override));
  MOCK_METHOD(int, setUvcExtensionUnit, (uint8_t, uint8_t, uint16_t, uint8_t*), (override));
  MOCK_METHOD(int, queueBuffer, (uint32_t), (override));
  MOCK_METHOD(int, dequeBuffer, (uint32_t&, timeval&), (override));
  MOCK_METHOD(int, startStream, (), (override));
  MOCK_METHOD(int, stopStream, (), (override));
  MOCK_METHOD(PollResult, poll, (int), (override));
  MOCK_METHOD(int, getControlValue, (uint32_t, int32_t&), (override));
  MOCK_METHOD(int, setControlValue, (uint32_t, int32_t), (override));
  MOCK_METHOD(int, requestBuffers, (uint32_t&), (override));
  MOCK_METHOD(bool, mmapBuffer, (uint32_t, nao_camera::ImageBuffer&), (override));
  MOCK_METHOD(bool, munmapBuffer, (nao_camera::ImageBuffer&), (override));
  MOCK_METHOD(std::vector<nao_camera::Control>, getControls, (), (const, override));
  // NOLINTEND(misc-non-private-member-variables-in-classes)
};

class NaoCameraDeviceSetupTest : public ::testing::Test {
protected:
  void SetUp() override {
    // for this test configure all methods to return the success value by default
    // so that in the tests we only need to explicitly set expectations for failure cases
    ON_CALL(mock_device_interface_, setImageFormat(testing::_, testing::_)).WillByDefault(testing::Return(0));
    ON_CALL(mock_device_interface_, setStreamParameters(testing::_)).WillByDefault(testing::Return(0));
    ON_CALL(mock_device_interface_, setUvcExtensionUnit(testing::_, testing::_, testing::_, testing::_))
      .WillByDefault(testing::Return(0));
    ON_CALL(mock_device_interface_, queueBuffer(testing::_)).WillByDefault(testing::Return(0));
    ON_CALL(mock_device_interface_, startStream()).WillByDefault(testing::Return(0));
    ON_CALL(mock_device_interface_, stopStream()).WillByDefault(testing::Return(0));
    ON_CALL(mock_device_interface_, mmapBuffer(testing::_, testing::_)).WillByDefault(testing::Return(true));
    ON_CALL(mock_device_interface_, munmapBuffer(testing::_)).WillByDefault(testing::Return(true));
    ON_CALL(mock_device_interface_, getControls()).WillByDefault(testing::Return(std::vector<nao_camera::Control>()));
    ON_CALL(mock_device_interface_, requestBuffers(testing::_)).WillByDefault(testing::Return(0));
  }

  // NOLINTBEGIN(misc-non-private-member-variables-in-classes)
  NiceMock<MockV4l2DeviceInterface> mock_device_interface_;
  // NOLINTEND(misc-non-private-member-variables-in-classes)
};

TEST_F(NaoCameraDeviceSetupTest, ThrowIfSetImageFormatFails) {
  EXPECT_CALL(mock_device_interface_, setImageFormat(testing::_, testing::_)).WillOnce(testing::Return(-1));
  EXPECT_THROW(NaoCameraDevice(mock_device_interface_, DEFAULT_SETTINGS), NaoCameraDeviceError);
}

TEST_F(NaoCameraDeviceSetupTest, ThrowIfSetStreamParametersFails) {
  EXPECT_CALL(mock_device_interface_, setStreamParameters(testing::_)).WillOnce(testing::Return(-1));
  EXPECT_THROW(NaoCameraDevice(mock_device_interface_, DEFAULT_SETTINGS), NaoCameraDeviceError);
}

TEST_F(NaoCameraDeviceSetupTest, ThrowIfSetUvcExtensionUnitFails) {
  EXPECT_CALL(mock_device_interface_, setUvcExtensionUnit(testing::_, testing::_, testing::_, testing::_))
    .WillOnce(testing::Return(-1));
  EXPECT_THROW(NaoCameraDevice(mock_device_interface_, DEFAULT_SETTINGS), NaoCameraDeviceError);
}

TEST_F(NaoCameraDeviceSetupTest, ThrowIfMemoryMappingFails) {
  EXPECT_CALL(mock_device_interface_, requestBuffers(testing::_)).WillOnce(testing::Return(-1));
  EXPECT_THROW(NaoCameraDevice(mock_device_interface_, DEFAULT_SETTINGS), NaoCameraDeviceError);

  EXPECT_CALL(mock_device_interface_, requestBuffers(testing::_)).WillOnce(testing::Return(0));
  EXPECT_CALL(mock_device_interface_, mmapBuffer(testing::_, testing::_))
    .Times(testing::AnyNumber())
    .WillRepeatedly(testing::Return(false));
  EXPECT_THROW(NaoCameraDevice(mock_device_interface_, DEFAULT_SETTINGS), NaoCameraDeviceError);
}

TEST_F(NaoCameraDeviceSetupTest, ThrowIfStartStreamFails) {
  EXPECT_CALL(mock_device_interface_, startStream()).WillOnce(testing::Return(-1));
  EXPECT_THROW(NaoCameraDevice(mock_device_interface_, DEFAULT_SETTINGS), NaoCameraDeviceError);
}

TEST_F(NaoCameraDeviceSetupTest, ThrowIfQueueBufferFails) {
  EXPECT_CALL(mock_device_interface_, queueBuffer(testing::_)).WillOnce(testing::Return(-1));
  EXPECT_THROW(NaoCameraDevice(mock_device_interface_, DEFAULT_SETTINGS), NaoCameraDeviceError);
}

class NaoCameraDeviceCaptureTest : public ::testing::Test {
protected:
  void SetUp() override {
    ON_CALL(mock_device_interface_, setImageFormat(testing::_, testing::_)).WillByDefault(testing::Return(0));
    ON_CALL(mock_device_interface_, setStreamParameters(testing::_)).WillByDefault(testing::Return(0));
    ON_CALL(mock_device_interface_, setUvcExtensionUnit(testing::_, testing::_, testing::_, testing::_))
      .WillByDefault(testing::Return(0));
    ON_CALL(mock_device_interface_, queueBuffer(testing::_)).WillByDefault(testing::Return(0));
    ON_CALL(mock_device_interface_, startStream()).WillByDefault(testing::Return(0));
    ON_CALL(mock_device_interface_, stopStream()).WillByDefault(testing::Return(0));
    ON_CALL(mock_device_interface_, getControls()).WillByDefault(testing::Return(std::vector<nao_camera::Control>()));
    ON_CALL(mock_device_interface_, requestBuffers(testing::_)).WillByDefault(testing::Return(0));
    ON_CALL(mock_device_interface_, munmapBuffer(testing::_)).WillByDefault(testing::Return(true));
    ON_CALL(mock_device_interface_, poll(testing::_)).WillByDefault(testing::Return(PollResult::OK));
    ON_CALL(mock_device_interface_, dequeBuffer(testing::_, testing::_))
      .WillByDefault(testing::Invoke([this](uint32_t& index, [[maybe_unused]] timeval&) {
        index = fake_buffers_.size() - 1;
        return 0;
      }));

    // mock mmap to actually allocate memory for the buffers using a vector default initialized to 0
    ON_CALL(mock_device_interface_, mmapBuffer(testing::_, testing::_))
      .WillByDefault(testing::Invoke([this]([[maybe_unused]] uint32_t index, nao_camera::ImageBuffer& buffer) {
        fake_buffers_.emplace_back(static_cast<std::size_t>(DEFAULT_SETTINGS.width * DEFAULT_SETTINGS.height * 2),
                                   static_cast<uint8_t>(0));
        buffer.data = fake_buffers_.back().data();

        return true;
      }));

    device_ = std::make_unique<NaoCameraDevice>(mock_device_interface_, DEFAULT_SETTINGS);
  }

  // NOLINTBEGIN(misc-non-private-member-variables-in-classes)
  NiceMock<MockV4l2DeviceInterface> mock_device_interface_;
  std::unique_ptr<NaoCameraDevice> device_;
  // NOLINTEND(misc-non-private-member-variables-in-classes)

private:
  std::vector<std::vector<uint8_t>> fake_buffers_;
};

TEST_F(NaoCameraDeviceCaptureTest, ReturnFalseIfPollFails) {
  EXPECT_CALL(mock_device_interface_, poll(testing::_)).WillOnce(testing::Return(PollResult::ERROR));

  sensor_msgs::msg::Image image;
  EXPECT_FALSE(device_->capture(image, 1000));
}

TEST_F(NaoCameraDeviceCaptureTest, ReturnFalseIfDequeueBufferFails) {
  EXPECT_CALL(mock_device_interface_, dequeBuffer(testing::_, testing::_)).WillOnce(testing::Return(-1));

  sensor_msgs::msg::Image image;
  EXPECT_FALSE(device_->capture(image, 1000));
}

TEST_F(NaoCameraDeviceCaptureTest, ThrowIfDequeueBufferReturnsInvalidIndex) {
  EXPECT_CALL(mock_device_interface_, dequeBuffer(testing::_, testing::_))
    .WillOnce(testing::Invoke([](uint32_t& index, timeval&) {
      index = 999;
      return 0;
    }));

  sensor_msgs::msg::Image image;
  EXPECT_THROW([[maybe_unused]] bool ret = device_->capture(image, 1000), NaoCameraDeviceError);
}

TEST_F(NaoCameraDeviceCaptureTest, CaptureSucceeds) {
  sensor_msgs::msg::Image image;
  ASSERT_TRUE(device_->capture(image, 1000));

  EXPECT_EQ(image.width, DEFAULT_SETTINGS.width);
  EXPECT_EQ(image.height, DEFAULT_SETTINGS.height);
  EXPECT_EQ(image.step, DEFAULT_SETTINGS.width * 2U);
  EXPECT_EQ(image.data.size(), DEFAULT_SETTINGS.width * DEFAULT_SETTINGS.height * 2U);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
