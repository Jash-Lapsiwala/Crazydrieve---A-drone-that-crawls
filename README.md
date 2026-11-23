# Crazydrieve---A-drone-that-crawls

```
 (\__/)  .~    ~. ))
 /O O  ./      .'
{O__,   \    {
  / .  . )    \
  |-| '-' \    }
 .(   _(   )_.'
'---.~_ _ _&
```
# Prerequisites:
- wsl (Ubuntu for compiling)
- Crazyflie environment setup (Link: https://www.bitcraze.io/documentation/tutorials/getting-started-with-crazyflie-2-x/)
- Crazyflie firmware (Link: https://github.com/bitcraze/crazyflie-firmware)
- Cflib (Link: https://github.com/bitcraze/crazyflie-lib-python)

# Compile Instructions:
## 1. In wsl:
- cd ~/crazyflie-firmware
- make --version
- make cf2_deconfig
- make -j$(nproc)
- ls build/*.bin

## 2. Build:
- ls -l build/cf2.bin
- cp build/cf2.bin /mnt/c/Users/lapsi/downloads/

## 3. Flash to the Firmware (in Downloads):
- cfloader flash C:\Users\lapsi\Downloads\cf2.bin stm32-fw

# Client (To check Battery status and Logging Files)
- Follow environment setup instructions
- Type 'client' in the directory where the environment is set up

# How to test?
- Go to the 'cflib_control.py' (you can ignore everything except for the main() function; this is where you make most of the changes)
- Run the file in a terminal inside the 'cflib_control.py' folder
- To fly the instructions are: 'Press Enter to fly (Ctrl+C to quit). During a flight, Space or Esc = KILL... Exiting'
- Note: you can run this from the git folder or where you saved the 'cflib_control.py' file
