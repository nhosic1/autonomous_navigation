#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <sys/mman.h>
#include <libcamera/libcamera.h>
#include "autonomous_navigation/stereo_processing.hpp"

using namespace libcamera;

class FramePublisher : public rclcpp::Node
{
public:
    FramePublisher() : Node("frame_publisher")
    {
        this->declare_parameter("camera_id", 0);
        camera_id_ = this->get_parameter("camera_id").as_int();
        std::string topic_name = "~/camera_" + std::to_string(camera_id_) + "/image";
        FramePublisher::image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name, 1);

        // Frame size params
        const int frame_width = 1536;
        const int frame_height = 864;

        cm_ = std::make_unique<CameraManager>();
        cm_->start();

        // Acquire the specified camera
        std::string camera_str_id = cm_->cameras()[camera_id_]->id();
        camera_ = cm_->get(camera_str_id);
        camera_->acquire();

        // Configure the camera with the StillCapture stream
        std::unique_ptr<CameraConfiguration> config = camera_->generateConfiguration({StreamRole::StillCapture});
        StreamConfiguration &stream_config = config->at(0);

        stream_config.size.width = frame_width;
        stream_config.size.height = frame_height;
        stream_config.pixelFormat = formats::BGR888;

        // Validate the config (original config might be modified, if invalid)
        config->validate();
        camera_->configure(config.get());

        std::cout << "Validated StillCapture configuration is: " << stream_config.toString() << std::endl;

        // Allocate frame buffers to makes sure the frames are not lost
        stream_ = stream_config.stream();
        allocator_ = new FrameBufferAllocator(camera_);
        allocator_->allocate(stream_);

        for (const std::unique_ptr<libcamera::FrameBuffer> &buffer : allocator_->buffers(stream_))
        {
            std::unique_ptr<libcamera::Request> request = camera_->createRequest();

            if (request->addBuffer(stream_, buffer.get()) < 0)
            {
                std::cerr << "Failed to set buffer for frame request" << std::endl;
            }

            requests_.push_back(std::move(request));
        }

        // Connect processing slot to requestCompleted signal
        camera_->requestCompleted.connect(this, &FramePublisher::process_request);

        // Start capturing frames
        camera_->start();
        for (std::unique_ptr<Request> &request : requests_)
            camera_->queueRequest(request.get());
    }

    ~FramePublisher()
    {
        camera_->stop();
        allocator_->free(stream_);
        delete allocator_;
        camera_->release();
        camera_.reset();
        cm_->stop();
    }

private:
    void process_request(Request *request)
    {
        if (request->status() == Request::RequestCancelled)
            return;

        const Request::BufferMap &buffers = request->buffers();
        for (auto buffer_pair : buffers)
        {
            const Stream *stream = buffer_pair.first;
            FrameBuffer *buffer = buffer_pair.second;
            const FrameMetadata &metadata = buffer->metadata();

            // Planes of the same buffer should use the same file descriptor
            size_t buffer_length = 0;
            int fd = -1;
            for (const libcamera::FrameBuffer::Plane &plane : buffer->planes())
            {
                buffer_length = std::max<size_t>(buffer_length, plane.offset + plane.length);
                if (fd == -1)
                    fd = plane.fd.get();
            }

            // Map the frame buffer planes into memory
            void *data = mmap(nullptr, buffer_length, PROT_READ, MAP_SHARED, fd, 0);

            // send image data
            std_msgs::msg::Header hdr;
            hdr.stamp = rclcpp::Time(int64_t(metadata.timestamp));
            hdr.frame_id = "camera_" + std::to_string(camera_id_);
            const libcamera::StreamConfiguration &stream_cfg = stream->configuration();

            auto msg_img = sensor_msgs::msg::Image();
            
            if (stream_cfg.pixelFormat.fourcc() == formats::BGR888.fourcc())
            {
                msg_img.header = hdr;
                msg_img.width = stream_cfg.size.width;
                msg_img.height = stream_cfg.size.height;
                msg_img.step = stream_cfg.stride;
                msg_img.encoding = sensor_msgs::image_encodings::RGB8;
                msg_img.is_bigendian = (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__);
                msg_img.data.resize(buffer_length);
                memcpy(msg_img.data.data(), data, buffer_length);
            }
            else
            {
                throw std::runtime_error("unsupported pixel format: " +
                                         stream->configuration().pixelFormat.toString());
            }

            // Publish image message
            image_publisher_->publish(msg_img);

            // Unmap the buffer
            munmap(data, buffer_length);
        }

        // Re-queue the request
        request->reuse(Request::ReuseBuffers);
        camera_->queueRequest(request);
    }

    int camera_id_;
    std::shared_ptr<Camera> camera_;
    std::unique_ptr<CameraManager> cm_;
    Stream *stream_;
    FrameBufferAllocator *allocator_;
    std::vector<std::unique_ptr<Request>> requests_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<FramePublisher>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}