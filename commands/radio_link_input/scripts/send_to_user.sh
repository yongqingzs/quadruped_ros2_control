#!/bin/bash

# 定义目标目录
TARGET_DIR="$HOME/Man/autostart"

# 如果目标目录不存在，则创建它
if [ ! -d "$TARGET_DIR" ]; then
    mkdir -p "$TARGET_DIR"
    echo "Created directory: $TARGET_DIR"
fi

# 复制文件到目标目录
if cp ./rc_controller_startup.py "$TARGET_DIR/"; then
    echo "Successfully copied rc_controller_startup.py to $TARGET_DIR/"
else
    echo "Error: Failed to copy rc_controller_startup.py" >&2
    exit 1
fi

if cp ./radio_link_input.py "$TARGET_DIR/"; then
    echo "Successfully copied radio_link_input.py to $TARGET_DIR/"
else
    echo "Error: Failed to copy radio_link_input.py" >&2
    exit 1
fi