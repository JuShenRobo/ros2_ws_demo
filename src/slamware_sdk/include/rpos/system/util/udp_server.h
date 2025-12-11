/**
* udp_server.h
*
* Copyright (c) 2024 Shanghai SlamTec Co., Ltd.
* Created By Yuqichai @ 2024-05-24
*/

/**
* Usage
*
* class SomeUdpServerHandler : public UdpServer<SomeUdpServerHandler>::IUdpServerHandler
* {
*     // Implement pure virutal functions
* };
*
* class SomeUdpServer : public UdpServer<SomeUdpServerHandler> {
*     // Write ctor & dtor
* };
*
* boost::shared_ptr<SomeUdpServer> server(new SomeUdpServer(boost::asio::ip::udp::endpoint(boost::asio::ip::v4(), 1314));
* server->start();
*/
#pragma once

#include <rpos/rpos_config.h>
#include <rpos/system/util/log.h>
#include <rpos/system/util/string_utils.h>
#include <rpos/system/target_info.h>
#include <rpos/system/this_thread.h>
#include <boost/noncopyable.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/atomic.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/bind.hpp>
#include <list>
#include <vector>
#include <string.h>

#define RPOS_SYSTEM_UTIL_UDPSERVER_DEFAULT_RX_BUFFER_SIZE 1024

namespace rpos { namespace system { namespace util {


    template <class UdpServerHandlerT, int RxBufferSize = RPOS_SYSTEM_UTIL_UDPSERVER_DEFAULT_RX_BUFFER_SIZE>
    class UdpServer : public boost::enable_shared_from_this<UdpServer<UdpServerHandlerT, RxBufferSize>>, private boost::noncopyable {
    public:
        typedef boost::shared_ptr<UdpServer<UdpServerHandlerT, RxBufferSize>> Pointer;

    public:
        class IUdpServerHandler {
        public:
            virtual ~IUdpServerHandler() {};
            virtual void onSendError(Pointer server, const boost::system::error_code& ec) = 0;
            virtual void onSendComplete(Pointer server) = 0;
            virtual void onReceiveError(Pointer server, const boost::system::error_code& ec, const boost::asio::ip::udp::endpoint& receiveEndpoint) = 0;
            virtual void onReceiveComplete(Pointer server, const unsigned char* buffer, size_t readBytes, const boost::asio::ip::udp::endpoint& receiveEndpoint) = 0; 
        };

        class EmptyUdpServerHandler : public IUdpServerHandler {
        public:
            virtual void onSendError(Pointer server, const boost::system::error_code& ec)
            {}

            virtual void onSendComplete(Pointer server)
            {}

            virtual void onReceiveError(Pointer server, const boost::system::error_code& ec, const boost::asio::ip::udp::endpoint& receiveEndpoint)
            {}

            virtual void onReceiveComplete(Pointer server, const unsigned char* buffer, size_t readBytes, const boost::asio::ip::udp::endpoint& receiveEndpoint)
            {} 
        };

    public:
        UdpServer(const boost::asio::ip::udp::endpoint& listenPoint)
            : io_()
            , socket_(io_, listenPoint)
            , handler_(new UdpServerHandlerT())
        {
            sending_ = false;
            receiving_ = false;
        }

        virtual ~UdpServer()
        {
            stop();

            if (ioThread_.joinable())
                ioThread_.join();

            handler_.reset();
        }

    public:
        void start()
        {
            boost::lock_guard<boost::mutex> guard(lock_);

            if (ioThread_.joinable())
                return;

            ioThread_ = boost::move(boost::thread(boost::bind(&UdpServer::worker_, this->shared_from_this()))); 
            
            startReceive_();
        }

        void stop()
        { 
            boost::lock_guard<boost::mutex> guard(lock_);
            if (!socket_.is_open())
                return;

            boost::system::error_code ec;
            socket_.shutdown(boost::asio::socket_base::shutdown_both, ec);
            socket_.close(ec); 
        }

        void send(const std::vector<unsigned char>& buffer, const boost::asio::ip::udp::endpoint& senderEndpoint)
        {
            if (buffer.empty())
                return;

            std::stringstream ss;
            ss << senderEndpoint.address().to_string() << ":" << senderEndpoint.port();
            std::string ipPortStr = ss.str();
            if (txBufferMaps_.find(ipPortStr) == txBufferMaps_.end())
                txBufferMaps_[ipPortStr] = std::vector<unsigned char>();

            {
                boost::lock_guard<boost::mutex> guard(sendLock_);

                if (txBufferMaps_[ipPortStr].empty())
                {
                    txBufferMaps_[ipPortStr] = buffer;
                }
                else
                {
                    size_t offset = txBufferMaps_[ipPortStr].size();
                    txBufferMaps_[ipPortStr].resize(offset + buffer.size());
                    memcpy(&txBufferMaps_[ipPortStr][offset], &buffer[0], buffer.size());
                }
            }
            startTransmit_(ipPortStr);
        }

        void stopWorkerThread()
        {
            if (ioThread_.joinable())
                ioThread_.join();
        }

        boost::asio::ip::udp::socket& socket()
        {
            return socket_;
        }

    private:
        // Send
        void startTransmit_(const std::string& ipPortStr)
        {
            {
                boost::lock_guard<boost::mutex> guard(sendLock_);

                if (sending_)
                    return;

                if (!txBufferMaps_[ipPortStr].size())
                    return;

                sending_ = true;
            }

            transmit_(ipPortStr);
        }

        void transmit_(const std::string& ipPortStr)
        {
            boost::lock_guard<boost::mutex> guard(sendLock_);

            transferredInBuffer_ = 0;
            transferingBuffer_ = txBufferMaps_[ipPortStr];
            nativeTxBuffer_ = &(transferingBuffer_)[0];
            txBufferMaps_[ipPortStr].clear();

            doTransmit_(ipPortStr);
        }

        void doTransmit_(const std::string& ipPortStr)
        {
            ipPortStr_ = ipPortStr;
            size_t pos = ipPortStr.find(":");
            std::string ip = ipPortStr.substr(0, pos);
            std::string port = ipPortStr.substr(pos + 1);
            boost::asio::ip::udp::endpoint sendEndpoint(boost::asio::ip::address::from_string(ip), std::stoi(port));
            socket_.async_send_to(
                boost::asio::buffer(nativeTxBuffer_ + transferredInBuffer_, transferingBuffer_.size() - transferredInBuffer_), sendEndpoint,
                boost::bind(&UdpServer::onTransmitComplete_, this->shared_from_this(), _1, _2)
                );
        }

        void onTransmitComplete_(const boost::system::error_code& ec, size_t written)
        {
            if (ec)
            {
                handler_->onSendError(this->shared_from_this(), ec);
                std::string tmpIpPortStr;
                {
                    boost::lock_guard<boost::mutex> guard(sendLock_);
                    sending_ = false;
                    for (auto iter = txBufferMaps_.begin(); iter != txBufferMaps_.end(); ++iter)
                    {
                        if (!iter->second.empty())
                        {
                            tmpIpPortStr = iter->first;
                            break;
                        }
                    }
                }
                if (!tmpIpPortStr.empty())
                    startTransmit_(tmpIpPortStr);
            }
            else
            {
                transferredInBuffer_ += written;

                if (transferredInBuffer_ == transferingBuffer_.size())
                {
                    {
                        boost::lock_guard<boost::mutex> guard(sendLock_);
                        sending_ = false;
                    }

                    if (txBufferMaps_[ipPortStr_].size())
                    {
                        return startTransmit_(ipPortStr_);
                    }

                    handler_->onSendComplete(this->shared_from_this());

                    std::string tmpIpPortStr;
                    {
                        boost::lock_guard<boost::mutex> guard(sendLock_);
                        for (auto iter = txBufferMaps_.begin(); iter != txBufferMaps_.end(); ++iter)
                        {
                            if (!iter->second.empty())
                            {
                                tmpIpPortStr = iter->first;
                                break;
                            }
                        }
                    }
                    if (!tmpIpPortStr.empty())
                        startTransmit_(tmpIpPortStr);
                }
                else
                {
                    doTransmit_(ipPortStr_);
                }
            }
        }

    private:
        // Receive
        void startReceive_()
        {
            bool expected = false;
            if(receiving_.compare_exchange_strong(expected,true))
                receive_(); 
        }

        void receive_()
        {
            socket_.async_receive_from(
                boost::asio::buffer(rxBlock_, RxBufferSize), receiveEndpoint_,
                boost::bind(&UdpServer::onReceiveComplete_, this->shared_from_this(), _1, _2)
                );
        }

        void onReceiveComplete_(const boost::system::error_code& ec, size_t readBytes)
        {
            if (ec)
            {
                if (ec != boost::asio::error::connection_refused)
                {
                    handler_->onReceiveError(this->shared_from_this(), ec, receiveEndpoint_);  
                }
            } 
            else if (readBytes)
            {
                handler_->onReceiveComplete(this->shared_from_this(), rxBlock_, readBytes, receiveEndpoint_);
            }

            receive_();
        }

    private:
        void worker_()
        {
            rpos::system::this_thread::setCurrentThreadName("UdpServer");
            while(1)
            {
                try
                {
                    io_.run();
                    break;
                }
                catch (const boost::system::system_error& )
                {
                    printf("UDP server IO error in udp server\n");
                }
            }
        }

    private:
        boost::asio::io_service io_;
        boost::thread ioThread_;
        boost::asio::ip::udp::socket socket_; 
        boost::mutex lock_;
        boost::asio::ip::udp::endpoint receiveEndpoint_;
        boost::scoped_ptr<UdpServerHandlerT> handler_;

        boost::atomic_bool sending_;
        boost::atomic_bool receiving_;

        boost::mutex sendLock_;
        std::map<std::string, std::vector<unsigned char>> txBufferMaps_;
        std::vector<unsigned char> transferingBuffer_;
        unsigned char* nativeTxBuffer_;
        size_t transferredInBuffer_;
        std::string ipPortStr_;

        unsigned char rxBlock_[RxBufferSize]; 
    };

} } }
