#ifndef UTIL_H
#define UTIL_H

#include <string>
#include <iostream>
#include <cstring>
#include <ctime>
#include <cerrno>

template<typename T> T get_ms(const struct timespec& ts)
{ return ts.tv_sec*1000 + ts.tv_nsec/(1000*1000); }

template<typename T> T get_ms(const struct timeval& ts)
{ return ts.tv_sec*1000 + ts.tv_usec/1000; }

float timediff_s(const struct timespec& t1, const struct timespec& t2);

int tcp_connect(const std::string& address);
int tcp_bind(const std::string& address);
int udp_connect(const std::string& address);
int udp_bind(const std::string& address);

bool fd_putline(int fd, const std::string& line, int eolchar = '\n');
bool fd_getline(int fd, std::string& line, int eolchar = '\n');

class Log
{
public:
    explicit Log(std::ostream& os) : mOs(os.rdbuf()) {}
    ~Log() { mOs << std::endl; }
    template<class T> Log& operator<<(const T& t)
    {
        mOs << t << ' ';
        return *this;
    }
private:
    std::ostream mOs;
};

struct NoLog
{
    NoLog() {}
    explicit NoLog(std::ostream&) {}
    template<class T> NoLog& operator<<(const T& t) { return *this; }
};

class Retval
{
public:
    explicit Retval(const std::string& desc = "") : mValue(0), mDesc(desc) {}
    Retval(const std::string& desc, int val) : mValue(0), mDesc(desc) { mValue = ThrowIfError(val); }
    Retval(int val) : mValue(0) { mValue = ThrowIfError(val); }
    virtual ~Retval() {}

    int operator=(int val) { return mValue = ThrowIfError(val); }
    int operator+=(int i) { return mValue += i; }
    int operator-=(int i) { return mValue -= i; }
    operator int() const { return mValue; }

protected:
    virtual std::string getString(int err) const { return strerror(err); }
    virtual int getError(int i) const { return i < 0 ? errno : 0; }

private:
    int ThrowIfError(int) const;

    int mValue;
    std::string mDesc;
};

struct FailIf : Retval
{
    FailIf(const std::string& desc = "") : Retval(desc) {}
    using Retval::operator=;
    using Retval::operator int;
    std::string getString(int) const override { return "Failed"; }
    int getError(int i) const override { return i; }
};

extern "C" const char* opus_strerror(int);
struct OpusRetval : Retval
{
    explicit OpusRetval(const std::string& desc = "") : Retval(desc) {}
    OpusRetval(int val) : Retval(val) {}
    using Retval::operator=;
    using Retval::operator+=;
    using Retval::operator-=;
    using Retval::operator int;
    std::string getString(int err) const override { return opus_strerror(err); }
    int getError(int val) const override { return val < 0 ? val : 0; }
};

#endif // UTIL_H
