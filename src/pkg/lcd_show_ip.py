#!/usr/bin/env python3
import socket
import subprocess
from rpi_lcd import LCD
import time

class NetworkDisplay:
    def __init__(self):
        self.lcd = LCD()
        self.prev_wifi = ""
        self.prev_ip = ""

    def get_wifi_name(self):
        """Lấy tên mạng WiFi hiện tại đang kết nối"""
        try:
            # Sử dụng iwgetid để lấy ESSID của mạng WiFi
            output = subprocess.check_output(['iwgetid', '-r']).decode('utf-8').strip()
            return output if output else "Not Connect WiFi"
        except (subprocess.CalledProcessError, FileNotFoundError):
            return "Not Connect WiFi"

    def get_ip_address(self):
        """Lấy địa chỉ IP hiện tại"""
        try:
            # Tạo một socket UDP và thử kết nối đến Google DNS
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return "Không có IP"

    def display_network_info(self):
        """Hiển thị thông tin mạng lên LCD"""
        try:
            wifi_name = self.get_wifi_name()
            ip_address = self.get_ip_address()
            
            # Chỉ cập nhật LCD khi thông tin thay đổi để giảm nhấp nháy
            if wifi_name != self.prev_wifi or ip_address != self.prev_ip:
                # Xóa màn hình
                self.lcd.clear()
                
                # Hiển thị tên WiFi ở dòng 1
                self.lcd.text(f"WiFi:{wifi_name}", 1)
                
                # Hiển thị địa chỉ IP ở dòng 2
                self.lcd.text(ip_address, 2)
                
                # Cập nhật giá trị trước đó
                self.prev_wifi = wifi_name
                self.prev_ip = ip_address
                
            return True
        except Exception as e:
            print(f"Error: {e}")

    def cleanup(self):
        try:
            self.lcd.clear()
        except:
            pass

def main():
    display = NetworkDisplay()
    try:
        while True:
            display.display_network_info()
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nĐang thoát...")
    finally:
        display.cleanup()
        print("Đã dọn dẹp màn hình LCD.")

if __name__ == "__main__":
    main()