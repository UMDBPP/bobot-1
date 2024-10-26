/** Includes */
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdlib.h>
#include <jpeglib.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

/** Image information */
#define ORIGINAL_WIDTH  640
#define ORIGINAL_HEIGHT 480
#define RESIZED_WIDTH   320
#define RESIZED_HEIGHT  240

/** Other names */
#define CAMERA_DEV      "/dev/video2"
#define IMAGE_HDR       "/home/romeperl/bobot-1/src/bobot_flight_images/images/image"

#define UCAM_SUCCESS 1

/** Internal error codes */
#define YUYV_RGB_CONV_FAILED   -1
#define JPG_COVERSION_FAILED   -2
#define RESIZE_FAILED          -3
#define INIT_FAILED            -4 
#define UCAM_START_FAILED      -5
#define UCAM_STOP_FAILED       -6
#define UCAM_NEXT_IMAGE_FAILED -7

/** Global variables */
int32_t camera_fd;
uint32_t counter;
void* input_buffer;
unsigned char rgb_buffer[ORIGINAL_WIDTH * ORIGINAL_HEIGHT * 3];
unsigned char resized_rgb_buffer[RESIZED_WIDTH * RESIZED_HEIGHT * 3];

struct v4l2_requestbuffers request_buffers;
struct v4l2_capability capability;
struct v4l2_buffer camera_buffer;
struct v4l2_format format;

rclcpp::Publisher<std_msgs::msg::String>::SharedPtr image_stream;
rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr image_stopper_serice;
rclcpp::TimerBase::SharedPtr timer;
rclcpp::Node::SharedPtr node;

void resize_rgb_image(unsigned char *src, unsigned char *dst, int src_width, int src_height, int dst_width, int dst_height);
int8_t save_as_jpeg(const char* filename, unsigned char* rgb_buffer, int width, int height);
void yuyv_to_rgb(unsigned char *yuyv, unsigned char *rgb, int width, int height);
int8_t get_next_image(const char* filename);
int8_t UCAM_LIB_StartStream(void);
int8_t UCAM_LIB_StopStream(void);
// int8_t UCAM_LIB_StopStream(void);
int32_t UCAM_LIB_Init(void);
void timer_callback(void);
int8_t prep_stream(void);
void call_stop_recording(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

std::string get_current_time()
    {
        // Stolen from Rahul
        std::ostringstream oss; // Make an ostringstream object
        auto t = std::time(nullptr); // Make a time objects
        auto tm = *std::localtime(&t); // Make a tm pointer thingy
        oss << std::put_time(&tm, "%Y-%m-%d"); // Do this stuff
        return oss.str(); // return da string
    }

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("bobot_camera");
    counter = 1;
    node->declare_parameter("CAMERA_FRAMES_PER_SEC", 24);

    int fps = node->get_parameter("CAMERA_FRAMES_PER_SEC").as_int();
    int freq_in_miliseconds = std::floor(1/fps * 1000);
    image_stream = node->create_publisher<std_msgs::msg::String>("/bobot_camera/heartbeat", 10);
    image_stopper_serice = node->create_service<std_srvs::srv::Trigger>(node->get_name() + std::string("/stop_recording"), &call_stop_recording);
    timer = node->create_wall_timer(std::chrono::milliseconds(freq_in_miliseconds), timer_callback);
    
    if(UCAM_LIB_Init() != UCAM_SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "[Bobot Camera] Failed to initialize oops baka.");
        rclcpp::shutdown();
        return 0;
    }
    
    if(UCAM_LIB_StartStream() != UCAM_SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "[Bobot Camera] Failed to start stream oops baka.");
        rclcpp::shutdown();
        return 0;
    }

    if(get_next_image("/home/romeperl/bobot-1/src/bobot_flight_images/images/image0.jpg") != UCAM_SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "[Bobot Camera] Failed to take initial image oops baka.");
        rclcpp::shutdown();
        return 0;
    }
    RCLCPP_ERROR(node->get_logger(), "[Bobot Camera] BALLS");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

/**
* service for start and stop using ucam_lib_start and stop commands 
* get_next_image() <- save image for u, just call with correct file path and name
* make sure that u are using counter to help generate new names
*/

void call_stop_recording(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    response->success = true;
    response->message = "Successfully stopped recording!";
    UCAM_LIB_StopStream();
    RCLCPP_INFO(node->get_logger(), "Successfully stopped recording!");
}

// In theory, we could add a start recording, but lets not for now? maybe later

void timer_callback(void) {
    std::string filename = IMAGE_HDR;
    filename.append(std::to_string(counter));
    filename.append(".jpg");

    if(get_next_image(filename.c_str()) != UCAM_SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "[Bobot Camera] Failed to take initial image oops baka.");
        rclcpp::shutdown();
        return;
    }

    counter += 1;
}

int32_t UCAM_LIB_Init(void) {
    if(prep_stream() != UCAM_SUCCESS){
        return -1;
    }
    RCLCPP_INFO(node->get_logger(), "[Bobot Camera] Initialized.");
    return UCAM_SUCCESS;
}
int8_t prep_stream(void){
    /** open the camera device */
    camera_fd = open(CAMERA_DEV, O_RDWR);
    if (camera_fd == -1) {
        RCLCPP_INFO(node->get_logger(), "[Bobot Camera] Opening video device ");
        return INIT_FAILED;
    } else {
        RCLCPP_INFO(node->get_logger(), "[Bobot Camera] Openned video device ");
    }

    /** collect capability data from stream */
    if (ioctl(camera_fd, VIDIOC_QUERYCAP, &capability) == -1) {
        RCLCPP_INFO(node->get_logger(), "[Bobot Camera] Failed to query capabilities ");
        return INIT_FAILED;
    } else {
        RCLCPP_INFO(node->get_logger(), "[Bobot Camera] Queried capabilities ");
    }

    /** set up format struct and send to camera */
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = ORIGINAL_WIDTH;
    format.fmt.pix.height = ORIGINAL_HEIGHT;
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    if (ioctl(camera_fd, VIDIOC_S_FMT, &format) == -1) {
        RCLCPP_INFO(node->get_logger(), "[Bobot Camera] Failed to set Pixel Format ");
        return INIT_FAILED;
    } else {
        RCLCPP_INFO(node->get_logger(), "[Bobot Camera] Set up pixel format ");
    }

    /** set up request buffer queue */
    request_buffers.count = 1;
    request_buffers.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    request_buffers.memory = V4L2_MEMORY_MMAP;
    if (ioctl(camera_fd, VIDIOC_REQBUFS, &request_buffers) == -1) {
        RCLCPP_INFO(node->get_logger(), "[Bobot Camera] Failed to make request buffer ");
        return INIT_FAILED;
    } else {
        RCLCPP_INFO(node->get_logger(), "[Bobot Camera] Made request buffer");
    }

    /** set up the input buffer itself */
    memset(&camera_buffer, 0, sizeof(camera_buffer));
    camera_buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    camera_buffer.memory = V4L2_MEMORY_MMAP;
    camera_buffer.index = 0;
    if (ioctl(camera_fd, VIDIOC_QUERYBUF, &camera_buffer) == -1) {
        RCLCPP_INFO(node->get_logger(), "[Bobot Camera] Failed to make query buffer ");
        return INIT_FAILED;
    } else {
        RCLCPP_INFO(node->get_logger(), "[Bobot Camera] Made query buffer ");
    }

    input_buffer = mmap(NULL, camera_buffer.length, PROT_READ | PROT_WRITE, MAP_SHARED, camera_fd, camera_buffer.m.offset);
    if (input_buffer == MAP_FAILED) {
        RCLCPP_INFO(node->get_logger(), "[Bobot Camera] Failed to make input buffer ");
        return INIT_FAILED;
    } else {
        RCLCPP_INFO(node->get_logger(), "[Bobot Camera] Made input buffer ");
    }
    return UCAM_SUCCESS;
}
int8_t UCAM_LIB_StartStream(void){
  if (ioctl(camera_fd, VIDIOC_STREAMON, &camera_buffer.type) == -1) {
      RCLCPP_INFO(node->get_logger(), "[Bobot Camera] Failed to start capturing ");
      return UCAM_START_FAILED;
  } else {
      RCLCPP_INFO(node->get_logger(), "[Bobot Camera] Started capturing images ");
  }
  return UCAM_SUCCESS;
}
int8_t UCAM_LIB_StopStream(void){
    if (ioctl(camera_fd, VIDIOC_STREAMOFF, &camera_buffer.type) == -1) {
        RCLCPP_INFO(node->get_logger(), "[Bobot Camera] Failed to stop capturing ");
        return UCAM_STOP_FAILED;
    } else {
        RCLCPP_INFO(node->get_logger(), "[Bobot Camera] Stopped capturing images ");
    }
    return UCAM_SUCCESS;
}
int8_t get_next_image(const char* filename){

    /** enqueue and dequeue the next image */
    if (ioctl(camera_fd, VIDIOC_QBUF, &camera_buffer) == -1) {
        RCLCPP_INFO(node->get_logger(), "[Bobot Camera] Failed to Queue Buffer ");
        return UCAM_NEXT_IMAGE_FAILED;
    }
    if (ioctl(camera_fd, VIDIOC_DQBUF, &camera_buffer) == -1) {
        RCLCPP_INFO(node->get_logger(), "[Bobot Camera] Failed to Dequeue Buffer ");
        return UCAM_NEXT_IMAGE_FAILED;
    }
    RCLCPP_ERROR(rclcpp::get_logger("BALLS"), "COCK AND BALLS");


    /** do meth for getting the right format */
    yuyv_to_rgb((unsigned char*)input_buffer, rgb_buffer, ORIGINAL_WIDTH, ORIGINAL_HEIGHT);
    resize_rgb_image(rgb_buffer, resized_rgb_buffer, ORIGINAL_WIDTH, ORIGINAL_HEIGHT, RESIZED_WIDTH, RESIZED_HEIGHT);

    /** save the image as a jpeg file */
    if(save_as_jpeg(filename, resized_rgb_buffer, RESIZED_WIDTH, RESIZED_HEIGHT) == JPG_COVERSION_FAILED){
        RCLCPP_INFO(node->get_logger(), "[Bobot Camera] Failed to Save Buffer ");  
        return UCAM_NEXT_IMAGE_FAILED;
    }
    
    return UCAM_SUCCESS;
}
int8_t save_as_jpeg(const char* filename, unsigned char* rgb_buffer, int width, int height) {
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;

    FILE *outfile = fopen(filename, "wb");
    if (!outfile) {
        RCLCPP_INFO(node->get_logger(), "[Bobot Camera] Failed to open %s to save image ", filename);
        return JPG_COVERSION_FAILED;
    }

    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);

    jpeg_stdio_dest(&cinfo, outfile);

    cinfo.image_width = width;
    cinfo.image_height = height;
    cinfo.input_components = 3;
    cinfo.in_color_space = JCS_RGB;

    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, 90, TRUE);

    jpeg_start_compress(&cinfo, TRUE);

    int row_stride = width * 3;
    while (cinfo.next_scanline < cinfo.image_height) {
        JSAMPROW row_pointer[1];
        row_pointer[0] = &rgb_buffer[cinfo.next_scanline * row_stride];
        jpeg_write_scanlines(&cinfo, row_pointer, 1);
    }

    jpeg_finish_compress(&cinfo);
    jpeg_destroy_compress(&cinfo);

    fclose(outfile);
    return UCAM_SUCCESS;
}
void resize_rgb_image(unsigned char *src, unsigned char *dst, int src_width, int src_height, int dst_width, int dst_height){
    float x_ratio = (float)src_width / dst_width;
    float y_ratio = (float)src_height / dst_height;

    for (int y = 0; y < dst_height; y++) {
        for (int x = 0; x < dst_width; x++) {
            int src_x = (int)(x * x_ratio);
            int src_y = (int)(y * y_ratio);

            int src_index = (src_y * src_width + src_x) * 3;
            int dst_index = (y * dst_width + x) * 3;

            dst[dst_index]     = src[src_index];
            dst[dst_index + 1] = src[src_index + 1];
            dst[dst_index + 2] = src[src_index + 2];
        }
    }
}
void yuyv_to_rgb(unsigned char *yuyv, unsigned char *rgb, int width, int height){
    int frame_size = width * height * 2;
    for (int i = 0; i < frame_size; i += 4) {
        unsigned char y0 = yuyv[i];
        unsigned char u  = yuyv[i + 1];
        unsigned char y1 = yuyv[i + 2];
        unsigned char v  = yuyv[i + 3];

        int c = y0 - 16;
        int d = u - 128;
        int e = v - 128;

        int r = (298 * c + 409 * e + 128) >> 8;
        int g = (298 * c - 100 * d - 208 * e + 128) >> 8;
        int b = (298 * c + 516 * d + 128) >> 8;

        rgb[i * 3 / 2]     = r > 255 ? 255 : r < 0 ? 0 : r;
        rgb[i * 3 / 2 + 1] = g > 255 ? 255 : g < 0 ? 0 : g;
        rgb[i * 3 / 2 + 2] = b > 255 ? 255 : b < 0 ? 0 : b;

        c = y1 - 16;
        r = (298 * c + 409 * e + 128) >> 8;
        g = (298 * c - 100 * d - 208 * e + 128) >> 8;
        b = (298 * c + 516 * d + 128) >> 8;

        rgb[i * 3 / 2 + 3] = r > 255 ? 255 : r < 0 ? 0 : r;
        rgb[i * 3 / 2 + 4] = g > 255 ? 255 : g < 0 ? 0 : g;
        rgb[i * 3 / 2 + 5] = b > 255 ? 255 : b < 0 ? 0 : b;
    }
}