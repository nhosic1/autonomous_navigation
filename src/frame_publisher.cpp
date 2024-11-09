#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <image_transport/image_transport.hpp>
#include <sys/mman.h>
#include <libcamera/libcamera.h>

using namespace libcamera;

class FramePublisher : public rclcpp::Node
{
public:
    FramePublisher() : Node("frame_publisher")
    {
        this->declare_parameter("camera_id", 0);
        camera_id_ = this->get_parameter("camera_id").as_int();

        // Frame size params
        const int frame_width = 768;
        const int frame_height = 432;

        cm_ = std::make_unique<CameraManager>();
        cm_->start();

        // Acquire the specified camera
        std::string camera_str_id = cm_->cameras()[camera_id_]->id();
        camera_ = cm_->get(camera_str_id);
        camera_->acquire();

        // Configure the camera with the StillCapture stream
        std::unique_ptr<CameraConfiguration> config = camera_->generateConfiguration({StreamRole::Viewfinder});
        StreamConfiguration &stream_config = config->at(0);

        stream_config.size.width = frame_width;
        stream_config.size.height = frame_height;
        stream_config.pixelFormat = formats::RGB888;

        // Rotate image
        config->orientation = Orientation::Rotate180;

        // Validate the config (original config might be modified, if invalid)
        config->validate();
        camera_->configure(config.get());

        RCLCPP_INFO(this->get_logger(), "Validated configuration: %s", stream_config.toString().c_str());

        // Allocate frame buffers to makes sure the frames are not lost
        stream_ = stream_config.stream();
        allocator_ = new FrameBufferAllocator(camera_);
        allocator_->allocate(stream_);

        int allocated_buffer_count = allocator_->buffers(stream_).size();
        RCLCPP_INFO(this->get_logger(), "Allocated buffers: %d", allocated_buffer_count);

        long min_frame_duration = 30000; // microseconds
        long max_frame_duration = 30000; // microseconds

        for (const std::unique_ptr<libcamera::FrameBuffer> &buffer : allocator_->buffers(stream_))
        {
            std::unique_ptr<libcamera::Request> request = camera_->createRequest();

            if (request->addBuffer(stream_, buffer.get()) < 0)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to set buffer for frame request");
            }

            // Set frame rate to 33.33 Hz
            request->controls().set(controls::FrameDurationLimits, Span<const std::int64_t, 2>({min_frame_duration, max_frame_duration}));

            // Disable autofocus
            request->controls().set(controls::AfMode, controls::AfModeManual);

            requests_.push_back(std::move(request));
        }

        float frame_rate = 1.0 / (static_cast<float>(min_frame_duration) / 1e6);
        RCLCPP_INFO(this->get_logger(), "Frame rate: %.2f Hz", frame_rate);

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

    void initialize_publishers(image_transport::ImageTransport &it)
    {
        // Create image publisher
        std::string topic_name = "~/camera_" + std::to_string(camera_id_) + "/image";
        image_publisher_ = it.advertise(topic_name, 1);
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

            if (stream_cfg.pixelFormat.fourcc() == formats::RGB888.fourcc())
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
                RCLCPP_ERROR(this->get_logger(), "Unsupported pixel format: %s", stream->configuration().pixelFormat.toString().c_str());
                rclcpp::shutdown();
            }

            // Publish image message
            image_publisher_.publish(msg_img);

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
    image_transport::Publisher image_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<FramePublisher>();

    image_transport::ImageTransport it(node);
    node->initialize_publishers(it);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}