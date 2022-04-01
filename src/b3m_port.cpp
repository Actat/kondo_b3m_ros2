#include <fcntl.h>
#include <string>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

class B3mPort
{
public:
  B3mPort(std::string device_name);
  ~B3mPort();
  bool readPort(void *buf, size_t count);
  bool writePort(char *buf, size_t count);

private:
  bool initialized_;
  std::string device_name_;
  int device_file_;
};

B3mPort::B3mPort(std::string device_name)
{
  initialized_ = false;
  device_name_ = device_name;
  device_file_ = open(device_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (device_file_ < 0)
  {
    throw exception("Could not open device file: " + device_name_);
  }
  initialized_ = true;
  return;
}

B3mPort::~B3mPort()
{
  if (initialized_)
  {
    close(device_file_);
    initialized_ = false;
  }
}

bool B3mPort::readPort(void *buf, size_t count)
{
  if (!initialized_)
  {
    return false;
  }
  ssize_t n_bytes_read = read(device_file_, buf, count);
  if (n_bytes_read == count)
  {
    return true;
  }
  else if (n_bytes_read < 0)
  {
    throw exception("Read error");
  }
  else
  {
    // number of bytes read is less than 'count'
    return false;
  }
}

bool B3mPort::writePort(char *buf, size_t count)
{
  if (!initialized_)
  {
    return false;
  }
  ssize_t written = write(device_file_, buf, count);
  if (written >= 0)
  {
    // success
    return true;
  }
  else if (errno == EWOULDBLOCK)
  {
    // write is blocked
    return false;
  }
  else
  {
    return false;
  }
}
