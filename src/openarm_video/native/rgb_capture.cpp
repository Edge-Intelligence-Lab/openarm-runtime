// Linux single-plane MJPEG owner. Preserve V4L2 timestamps, never substitute read time.
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

static void call(int fd, unsigned long op, void* p) {
    int r; do {r=ioctl(fd,op,p);} while(r<0 && errno==EINTR);
    if(r<0) throw std::runtime_error(std::string("V4L2 ioctl: ")+strerror(errno));
}
static void write_all(const void* data, size_t length) {
    auto p=static_cast<const char*>(data);
    while(length) {auto n=write(STDOUT_FILENO,p,length);
        if(n<0 && errno==EINTR) continue;
        if(n<=0) throw std::runtime_error("RGB output closed");
        p+=n;length-=n;
    }
}
struct Mapping {void* data; size_t length;};
int main(int argc,char** argv) {
    int fd=-1;bool streaming=false;std::vector<Mapping> maps;
    auto cleanup=[&] {
        if(streaming) {int type=V4L2_BUF_TYPE_VIDEO_CAPTURE;ioctl(fd,VIDIOC_STREAMOFF,&type);}
        for(auto m:maps) munmap(m.data,m.length);
        if(fd>=0) close(fd);
    };
    try {
        if(argc!=5) throw std::runtime_error("device width height fps required");
        unsigned w=std::stoul(argv[2]),h=std::stoul(argv[3]),fps=std::stoul(argv[4]);
        if(!w||!h||w>4096||h>4096||!fps||fps>120) throw std::runtime_error("Invalid RGB mode");
        fd=open(argv[1],O_RDWR|O_NONBLOCK|O_CLOEXEC);
        if(fd<0) throw std::runtime_error("Cannot open RGB device");
        v4l2_capability cap{};call(fd,VIDIOC_QUERYCAP,&cap);
        unsigned caps=(cap.capabilities&V4L2_CAP_DEVICE_CAPS)?cap.device_caps:cap.capabilities;
        if(!(caps&V4L2_CAP_VIDEO_CAPTURE)||!(caps&V4L2_CAP_STREAMING)) throw std::runtime_error("MMAP RGB capture unsupported");
        v4l2_format fmt{};fmt.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width=w;fmt.fmt.pix.height=h;fmt.fmt.pix.pixelformat=V4L2_PIX_FMT_MJPEG;
        fmt.fmt.pix.field=V4L2_FIELD_NONE;call(fd,VIDIOC_S_FMT,&fmt);
        if(fmt.fmt.pix.width!=w||fmt.fmt.pix.height!=h||fmt.fmt.pix.pixelformat!=V4L2_PIX_FMT_MJPEG)
            throw std::runtime_error("Driver changed requested native MJPEG mode");
        v4l2_streamparm parm{};parm.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
        parm.parm.capture.timeperframe.numerator=1;parm.parm.capture.timeperframe.denominator=fps;
        call(fd,VIDIOC_S_PARM,&parm);call(fd,VIDIOC_G_PARM,&parm);
        auto period=parm.parm.capture.timeperframe;
        if(!period.denominator||!period.numerator) throw std::runtime_error("Missing driver frame period");
        double seconds=double(period.numerator)/period.denominator;
        if(seconds>0.1||seconds<1./120.) throw std::runtime_error("Unsupported actual frame period");
        v4l2_requestbuffers req{};req.count=4;req.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;req.memory=V4L2_MEMORY_MMAP;
        call(fd,VIDIOC_REQBUFS,&req);
        if(req.count<2||req.count>32) throw std::runtime_error("Invalid driver buffer count");
        for(unsigned i=0;i<req.count;i++) {
            v4l2_buffer b{};b.type=req.type;b.memory=req.memory;b.index=i;call(fd,VIDIOC_QUERYBUF,&b);
            if(!b.length||b.length>24000000) throw std::runtime_error("Unbounded driver buffer");
            void* p=mmap(nullptr,b.length,PROT_READ|PROT_WRITE,MAP_SHARED,fd,b.m.offset);
            if(p==MAP_FAILED) throw std::runtime_error("RGB mmap failed");
            maps.push_back({p,b.length});call(fd,VIDIOC_QBUF,&b);
        }
        int type=V4L2_BUF_TYPE_VIDEO_CAPTURE;call(fd,VIDIOC_STREAMON,&type);streaming=true;
        for(;;) {
            fd_set fds;FD_ZERO(&fds);FD_SET(fd,&fds);timeval timeout{2,0};
            int ready=select(fd+1,&fds,nullptr,nullptr,&timeout);
            if(ready<0 && errno==EINTR) continue;
            if(ready<=0) throw std::runtime_error("RGB capture timed out");
            v4l2_buffer b{};b.type=req.type;b.memory=req.memory;
            call(fd,VIDIOC_DQBUF,&b);
            if(b.index>=maps.size()||!b.bytesused||b.bytesused>maps[b.index].length||b.flags&V4L2_BUF_FLAG_ERROR)
                throw std::runtime_error("Invalid RGB buffer");
            // Exposure is read, never changed. Unknown exposure leaves preview
            // available but cannot qualify an automatic-calibration sample.
            v4l2_control exposure{};exposure.id=V4L2_CID_EXPOSURE_ABSOLUTE;
            bool known=ioctl(fd,VIDIOC_G_CTRL,&exposure)==0 && exposure.value>0 && exposure.value<=2000;
            double stamp=b.timestamp.tv_sec+b.timestamp.tv_usec/1000000.;
            char header[1024];
            int size=snprintf(header,sizeof(header),
                "{\"bytes\":%u,\"kernel_sequence\":%u,\"timestamp_s\":%.6f,\"flags\":%u,"
                "\"period_s\":%.9f,\"exposure_100us\":%d,\"exposure_readable\":%s,"
                "\"driver_version\":%u,\"width\":%u,\"height\":%u}\n",
                b.bytesused,b.sequence,stamp,b.flags,seconds,exposure.value,known?"true":"false",cap.version,w,h);
            if(size<=0||size>=int(sizeof(header))) throw std::runtime_error("RGB header overflow");
            write_all(header,size);write_all(maps[b.index].data,b.bytesused);
            call(fd,VIDIOC_QBUF,&b);
        }
    } catch(const std::exception& e) {
        fprintf(stderr,"%s\n",e.what());cleanup();return 1;
    }
}
