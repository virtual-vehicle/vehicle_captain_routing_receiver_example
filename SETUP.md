# Setup Guide
Please read the [README](https://github.com/virtual-vehicle/vehicle_captain_routing_receiver_example/blob/master/README.md) first.

Use the [QUICKSTART](https://github.com/virtual-vehicle/vehicle_captain_routing_receiver_example/blob/master/QUICKSTART.md) guide for local testing.

## 1. General configuration
### 1.1. Ports
- Receiver Port (from vehicleCAPTAIN routing core): 5555
- Sender Port (from vehicleCAPTAIN routing core): 5556
- Local Exchange Port (between sender and receiver examples): 5557

BE AWARE: the vehicleCAPTAIN routing core receives/sends messages via its connected V2X interfaces. There is no direct feedback loop.

### 1.2. Run RECEIVER Demo Software
The IP is either:
- IP of the receiving routing core (receiving vehicleCAPTAIN)
- IP of the sender demo (127.0.0.1 with network bridge enabled)
- 
  ```bash
    ./docker_demo_build_and_run
    docker exec cpp_receiver-demo ./cppReceiver local <ip> <port>
  ```

## 2. Development Environments
The dev docker can be used for development. Below are two examples on how to integrate it with CLion.
Other IDEs may be configured similarly.

### 2.1 Option 1: Remote Host
This configuration can be used when the IDE supports SSH remote deployment.

#### 2.1.1 RECEIVER Setup
- git clone CPP-Receiver repo
- in root folder execute docker script:
  ```bash
  ./docker_dev_build_and_run.sh
  ```

#### 2.1.2 Configurations in CLion
- the docker has to be running, otherwise the connection is going to be refused
- make the following configurations either for sender or receiver

  ##### Toolchain Configuration
  - File -> Settings -> Build, Execution, Deployment -> Toolchains
  - Click "+" and "New Remote Host"
  - Configure new remote (SSH Configurations):
    - Host: localhost
    - Port: 2222
    - Username: user
    - Password: password

  ##### CMake Configuration
  - File -> Settings -> Build, Execution, Deployment -> CMake
  - Click "+" and name it "Debug-Remote" for example
  - Select the previously created toolchain
  - Reload the CMake Project (open CMake file in CLion and click Reload CMake project)

#### 2.1.3 RECEIVING messages
```bash
  docker exec -it cpp_receiver-dev ./cppReceiver <ip> <port>
```

### 2.2 Option 2: Docker Connection
This configuration can be used if the IDE allows direct docker access.

#### 2.2.1 RECEIVER Setup
- git clone CPP-Receiver repo
- in root folder execute docker script:
  ```bash
  ./docker_dev_build_and_run.sh
  ```

#### 2.2.2 Configurations in CLion

##### Toolchain Configuration
- File -> Settings -> Build, Execution, Deployment -> Toolchains
- Click "+" and "Docker"
- Setting should look like this:
  - Server: Docker
  - Image: clion/lw_remote-cpp_receiver-env:0.1
- Click 'Apply' and 'OK'.

##### CMake Configuration
- File -> Settings -> Build, Execution, Deployment -> CMake
- Click "+" and name it "Debug-Docker" for example
- Select the previously created toolchain
- Reload the CMake Project (open CMake file in CLion and click Reload CMake project)

#### 2.2.3 RECEIVING messages
```bash
  docker exec -it cpp_receiver-dev ./cppReceiver <ip> <port>
```

In summary, a script creates a Docker with a working configuration, that can be connected to via SSH.

## 3. Hints for consecutive tests
- the cpp_receiver-demo docker has to run
```bash
docker start cpp_receiver-demo
```
- if the code has changed, the docker has to be built again
```bash
docker rm cpp_receiver-demo
./docker_demo_build_and_run.sh
```
- if the docker is built and running
```bash
docker exec cpp_receiver-demo ./cppReceiver <ip> <port>
```

- if the ip address is already in use:
```bash
docker stop cpp_receiver-demo
docker start cpp_receiver-demo
```