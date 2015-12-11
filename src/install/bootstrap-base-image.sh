#!/bin/bash
# Run this as root from inside fresh ODROID base image on hardware

# Install Docker and get it up to version 1.9.0
apt-get update
apt-get install -y lxc aufs-tools cgroup-lite apparmor docker.io
service docker stop
cd /usr/bin
mv docker docker.old
wget https://github.com/umiddelb/armhf/raw/master/bin/docker-1.9.0
chmod a+x docker-1.9.0
ln -s docker-1.9.0 docker
service docker restart

# Throttle back Cortex-A15 CPU cores due to heat issues and to save power
# Cores 0-3 are Cortex-A7 1.4GHz max and [4-7] are Cortex-A15 2.0GHz max
for x in /sys/devices/system/cpu/cpu{0..7}/cpufreq/scaling_max_freq; do
    echo 1400000 > $x
done

# To persist this change across reboots throw it into /etc/rc.local
# first change /etc/rc.local to use bash instead of sh
sed -i '1s/sh -e/bash/' /etc/rc.local
sed -i '/^exit 0/i \
for x in /sys/devices/system/cpu/cpu{0..7}/cpufreq/scaling_max_freq; do \
    echo 1400000 > $x \
done \
' /etc/rc.local

# References for various governor modes:
# https://android.googlesource.com/kernel/common/+/a7827a2a60218b25f222b54f77ed38f57aebe08b/Documentation/cpu-freq/governors.txt
# Confirm kernel has CPU Frequency Governor support and is in interactive mode
grep CPU_FREQ_GOV /boot/config-$(uname -r)
cat /sys/devices/system/cpu/cpu{0..7}/cpufreq/scaling_available_governors 
cat /sys/devices/system/cpu/cpu{0..7}/cpufreq/scaling_governor
# Now set both Cortex-A7 and Cortex-A15 to *interactive* mode
# Check current speed of all cores
cat /sys/devices/system/cpu/cpu{0..7}/cpufreq/cpuinfo_cur_freq

# Now check temperatures from various onboard sensors
# can also use `watch` to monitor temps
cat /sys/devices/10060000.tmu/temp
