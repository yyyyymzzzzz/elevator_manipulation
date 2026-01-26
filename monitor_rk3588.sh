#!/bin/bash
while true
do
    clear
    echo "========================================"
    echo "       RK3588 Resource Monitor"
    echo "========================================"
    echo ""
    
    # 1. CPU Temp (CPU温度)
    cpu_temp=$(cat /sys/class/thermal/thermal_zone0/temp)
    echo "CPU Temp: $(($cpu_temp/1000))°C"
    echo ""

    # 2. CPU Freq (CPU频率 - 取大核 Core4 和 小核 Core0)
    freq_big=$(cat /sys/devices/system/cpu/cpu4/cpufreq/scaling_cur_freq)
    freq_little=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq)
    echo "CPU Freq (Big): $(($freq_big/1000)) MHz"
    echo "CPU Freq (Little): $(($freq_little/1000)) MHz"
    echo ""

    # 3. NPU Load
    echo "--- NPU Load ---"
    if [ -f /sys/kernel/debug/rknpu/load ]; then
        cat /sys/kernel/debug/rknpu/load
    else
        echo "NPU driver debug file not found (try sudo)"
    fi
    echo ""

    # 4. GPU Load
    echo "--- GPU Load ---"
    if [ -f /sys/devices/platform/fb000000.gpu/utilisation ]; then
        util=$(cat /sys/devices/platform/fb000000.gpu/utilisation)
        echo "GPU Usage: $util %"
    else
        echo "GPU path not found"
    fi
    
    sleep 0.5
done