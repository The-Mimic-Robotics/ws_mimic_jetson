#!/usr/bin/env python3
"""
ODOM Serial Monitor - Debug ESP32 odometry messages
Monitors raw serial data from ESP32 and validates REP-103 compliance
"""

import serial
import time
import sys
import math

class OdomMonitor:
    def __init__(self, port='/dev/ttyUSB0', baud=115200):
        self.port = port
        self.baud = baud
        
        # Statistics
        self.message_count = 0
        self.parse_errors = 0
        self.last_message_time = time.time()
        
        # Previous values for delta calculation
        self.prev_x = None
        self.prev_y = None
        self.prev_theta = None
        self.prev_time = None
        
        print(f"🔍 ODOM Serial Monitor Starting...")
        print(f"Port: {port} @ {baud} baud")
        print(f"Expected format: ODOM,x,y,theta,vx,vy,omega,enc1,enc2,enc3,enc4")
        print("=" * 80)
        
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            time.sleep(2)  # Let ESP32 stabilize
            print("✅ Serial connection established")
            print("=" * 80)
        except Exception as e:
            print(f"❌ Failed to open serial port: {e}")
            sys.exit(1)
    
    def parse_odom(self, line):
        """Parse ODOM message and validate"""
        try:
            parts = line.split(',')
            
            if len(parts) != 11:
                self.parse_errors += 1
                print(f"❌ Invalid format (expected 11 fields, got {len(parts)}): {line}")
                return None
            
            # Parse all fields
            data = {
                'x': float(parts[1]),
                'y': float(parts[2]),
                'theta': float(parts[3]),
                'vx': float(parts[4]),
                'vy': float(parts[5]),
                'omega': float(parts[6]),
                'enc_fl': int(parts[7]),
                'enc_fr': int(parts[8]),
                'enc_bl': int(parts[9]),
                'enc_br': int(parts[10]),
                'timestamp': time.time()
            }
            
            self.message_count += 1
            return data
            
        except (ValueError, IndexError) as e:
            self.parse_errors += 1
            print(f"❌ Parse error: {e}")
            print(f"   Raw line: {line}")
            return None
    
    def calculate_deltas(self, data):
        """Calculate position/velocity changes"""
        if self.prev_x is None:
            self.prev_x = data['x']
            self.prev_y = data['y']
            self.prev_theta = data['theta']
            self.prev_time = data['timestamp']
            return None
        
        dt = data['timestamp'] - self.prev_time
        if dt < 0.001:  # Avoid division by zero
            return None
        
        deltas = {
            'dx': data['x'] - self.prev_x,
            'dy': data['y'] - self.prev_y,
            'dtheta': data['theta'] - self.prev_theta,
            'dt': dt,
            'calc_vx': (data['x'] - self.prev_x) / dt,
            'calc_vy': (data['y'] - self.prev_y) / dt,
            'calc_omega': (data['theta'] - self.prev_theta) / dt
        }
        
        # Update previous values
        self.prev_x = data['x']
        self.prev_y = data['y']
        self.prev_theta = data['theta']
        self.prev_time = data['timestamp']
        
        return deltas
    
    def validate_data(self, data):
        """Validate REP-103 compliance and sanity"""
        issues = []
        
        # Check for reasonable position values (within +/- 100m)
        if abs(data['x']) > 100 or abs(data['y']) > 100:
            issues.append(f"⚠️  Large position: x={data['x']:.3f}, y={data['y']:.3f}")
        
        # Check for reasonable velocity values (< 2 m/s for mecanum)
        if abs(data['vx']) > 2.0 or abs(data['vy']) > 2.0:
            issues.append(f"⚠️  High velocity: vx={data['vx']:.3f}, vy={data['vy']:.3f}")
        
        # Check for reasonable angular velocity (< 3 rad/s)
        if abs(data['omega']) > 3.0:
            issues.append(f"⚠️  High omega: {data['omega']:.3f} rad/s")
        
        # Check encoder values are changing
        if all(enc == 0 for enc in [data['enc_fl'], data['enc_fr'], data['enc_bl'], data['enc_br']]):
            issues.append("⚠️  All encoders are zero")
        
        return issues
    
    def print_summary(self, data, deltas):
        """Print formatted summary of odometry data"""
        print(f"\n📊 Message #{self.message_count} | Errors: {self.parse_errors}")
        print("─" * 80)
        
        # Position
        print(f"📍 Position (REP-103):")
        print(f"   X (forward):     {data['x']:>8.4f} m")
        print(f"   Y (left):        {data['y']:>8.4f} m")
        print(f"   Theta (heading): {data['theta']:>8.4f} rad ({math.degrees(data['theta']):>7.2f}°)")
        
        # Velocity (reported by ESP32)
        print(f"\n🚀 Velocity (ESP32 reported):")
        print(f"   Vx (forward):    {data['vx']:>8.4f} m/s")
        print(f"   Vy (left):       {data['vy']:>8.4f} m/s")
        print(f"   Omega (CCW):     {data['omega']:>8.4f} rad/s")
        
        # Deltas and calculated velocity
        if deltas:
            print(f"\n📈 Calculated from position delta (dt={deltas['dt']:.3f}s):")
            print(f"   ΔX: {deltas['dx']:>8.4f} m  → Calc Vx: {deltas['calc_vx']:>8.4f} m/s")
            print(f"   ΔY: {deltas['dy']:>8.4f} m  → Calc Vy: {deltas['calc_vy']:>8.4f} m/s")
            print(f"   Δθ: {deltas['dtheta']:>8.4f} rad → Calc ω:  {deltas['calc_omega']:>8.4f} rad/s")
            
            # Check consistency
            vx_diff = abs(data['vx'] - deltas['calc_vx'])
            vy_diff = abs(data['vy'] - deltas['calc_vy'])
            omega_diff = abs(data['omega'] - deltas['calc_omega'])
            
            if vx_diff > 0.1 or vy_diff > 0.1 or omega_diff > 0.1:
                print(f"\n   ⚠️  Velocity mismatch detected!")
                print(f"      Vx diff: {vx_diff:.4f}, Vy diff: {vy_diff:.4f}, ω diff: {omega_diff:.4f}")
        
        # Encoders
        print(f"\n🔢 Encoder Counts:")
        print(f"   FL: {data['enc_fl']:>7}  |  FR: {data['enc_fr']:>7}")
        print(f"   BL: {data['enc_bl']:>7}  |  BR: {data['enc_br']:>7}")
        
        # Validation
        issues = self.validate_data(data)
        if issues:
            print(f"\n⚠️  Issues Found:")
            for issue in issues:
                print(f"   {issue}")
        else:
            print(f"\n✅ All values within normal range")
        
        print("═" * 80)
    
    def run(self, verbose=True, interval=1.0):
        """Main monitoring loop"""
        print("\n🎬 Monitoring started (Ctrl+C to stop)...")
        print(f"Display interval: {interval}s | Verbose: {verbose}\n")
        
        last_display = time.time()
        latest_data = None
        latest_deltas = None
        
        try:
            while True:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    if line.startswith('ODOM,'):
                        data = self.parse_odom(line)
                        if data:
                            deltas = self.calculate_deltas(data)
                            latest_data = data
                            latest_deltas = deltas
                            
                            # Display at specified interval
                            if time.time() - last_display >= interval:
                                self.print_summary(data, deltas)
                                last_display = time.time()
                    
                    elif line and verbose:
                        # Show other ESP32 messages
                        print(f"💬 ESP32: {line}")
                
                time.sleep(0.001)  # Small delay to prevent CPU spinning
                
        except KeyboardInterrupt:
            print("\n\n🛑 Monitoring stopped")
            print(f"\n📊 Final Statistics:")
            print(f"   Total messages: {self.message_count}")
            print(f"   Parse errors:   {self.parse_errors}")
            if self.message_count > 0:
                print(f"   Success rate:   {100 * (1 - self.parse_errors/max(self.message_count, 1)):.1f}%")
            
            if latest_data:
                print(f"\n📍 Last Position: x={latest_data['x']:.4f}, y={latest_data['y']:.4f}, θ={latest_data['theta']:.4f}")
                print(f"🚀 Last Velocity: vx={latest_data['vx']:.4f}, vy={latest_data['vy']:.4f}, ω={latest_data['omega']:.4f}")
        
        finally:
            self.ser.close()
            print("\n✅ Serial connection closed")

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Monitor ESP32 ODOM messages')
    parser.add_argument('--port', default='/dev/ttyUSB0', help='Serial port (default: /dev/ttyUSB0)')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate (default: 115200)')
    parser.add_argument('--interval', type=float, default=1.0, help='Display interval in seconds (default: 1.0)')
    parser.add_argument('--verbose', action='store_true', help='Show all ESP32 messages')
    
    args = parser.parse_args()
    
    monitor = OdomMonitor(port=args.port, baud=args.baud)
    monitor.run(verbose=args.verbose, interval=args.interval)

if __name__ == '__main__':
    main()
