/****************************************************************************
*
* Copyright (C) 2016 Emil Fresk.
* All rights reserved.
*
* This file is part of the ROS ViconStream node.
*
* GNU Lesser General Public License Usage
* This file may be used under the terms of the GNU Lesser
* General Public License version 3.0 as published by the Free Software
* Foundation and appearing in the file LICENSE included in the
* packaging of this file.  Please review the following information to
* ensure the GNU Lesser General Public License version 3.0 requirements
* will be met: http://www.gnu.org/licenses/lgpl-3.0.html.
*
* If you have questions regarding the use of this file, please contact
* Emil Fresk at emil.fresk@gmail.com.
*
****************************************************************************/

#ifndef _DEADLINE_H
#define _DEADLINE_H

#include <map>
#include <vector>
#include <thread>
#include <mutex>
#include <chrono>

namespace Deadline
{

class Deadline
{
private:

    unsigned int _time, _time_to_wait;
    std::function<void(void)> _callback;

    std::thread _deadline_thread;
    bool _shutdown;
    std::mutex _access;
    bool _active;

    void deadlineWorker(void)
    {
        while (!_shutdown)
        {
            std::unique_lock<std::mutex> locker(_access);

            if (_active)
            {
                _time++;

                if (_time > _time_to_wait)
                {
                    _callback();

                    _active = false;
                }
            }

            locker.unlock();

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

public:
    Deadline(std::function<void(void)> callback, unsigned int deadline_ms)
        : _time_to_wait(deadline_ms), _callback(callback), _shutdown(false),
          _active(false)
    {

        _deadline_thread = std::thread(&Deadline::deadlineWorker, this);

    }

    ~Deadline()
    {
        _shutdown = true;
        _deadline_thread.join();
    }

    bool setDeadline(unsigned int deadline_ms)
    {
        std::lock_guard<std::mutex> locker(_access);

        if (!_active)
            _time_to_wait = deadline_ms;

        return !_active;
    }

    void start()
    {
        std::lock_guard<std::mutex> locker(_access);

        _time = 0;
        _active = true;
    }

    void stop()
    {
        std::lock_guard<std::mutex> locker(_access);

        _time = 0;
        _active = false;
    }
};

}

#endif
