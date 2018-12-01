#!/bin/bash

echo 1 | sudo tee /sys/bus/usb-serial/devices/$1/latency_timer
