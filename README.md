# beckhoff_ads_hardware_interface for ROS 2
Copyright (c) 2025, b-robotized. All rights reserved.
Author: Nikola Banovic Contributor: Hajar Bartakh

This package provides a `ros2_control` **SystemInterface** for communicating with Beckhoff TwinCAT PLCs. It acts as a transport layer, allowing `ros2_control` controllers to read from and write to PLC variables (e.g., joint states, GPIOs, sensor values) over the network.

The core of this hardware interface is in using **ADS Sum-Commands**. This allows us to read/write multiple PLC variables in a single network transaction, bringing the total network transactions per update loop to only 2 (one **read** and one **write**).

This package is built upon the [official `beckhoff/ADS` library](https://github.com/Beckhoff/ADS), which handles the low-level ADS protocol communication.

---

## Key Features
* **`ros2_control` Integration**: Seamlessly integrate your PLC application with the `ros2_control` framework.
* **Efficient Communication**: Utilizes ADS Sum-Commands to bundle multiple variable requests, ensuring high-performance communication suitable for real-time control loops.
* **URDF-Based Configuration**: All hardware connections and variable mappings are configured directly within your robot's URDF file.

---

## Requirements

* ROS 2 (Jazzy Jalisco or newer recommended)
* [ads_vendor](https://github.com/b-robotized/ads_vendor) package


## Configuration
Configuration is managed entirely within the `<ros2_control>` tag of your robot's URDF file. You must specify the PLC connection parameters and map each hardware interface (state or command) to a specific variable on the PLC.

### 1. Hardware Parameters
These parameters define the connection to the target PLC.

| Parameter          | Type     | Description                                     |
| :----------------- | :------- | :---------------------------------------------- |
| `plc_ip_address`   | `string` | The IP address of the Beckhoff PLC.             |
| `plc_ams_net_id`   | `string` | The AMS NetID of the target PLC (e.g., "192.168.1.1.1.1"). |
| `local_ams_net_id` | `string` | The AMS NetID of the computer running ROS.      |
| `plc_ams_port`     | `string` | The AMS Port of the PLC runtime (e.g., "851").  |
| `read_mode`        | `string` | (Optional) `notification` (default) reads state via PLC-pushed ADS device notifications cached for a non-blocking `read()`. `polling` uses the synchronous ADS Sum-Read every cycle. Case-insensitive. |

### 2. Interface Parameters
For each `<state_interface>` and `<command_interface>`, you must provide parameters that link it to a PLC variable.

| Parameter       | Type      | Description                                                                 |
| :-------------- | :-------- | :-------------------------------------------------------------------------- |
| `PLC_symbol`    | `string`  | The full symbolic name of the variable in the PLC (e.g., "MAIN.Joint_Pos_State"). |
| `PLC_type`      | `string`  | The data type of the PLC variable (e.g., "LREAL", "BOOL", "DINT"). Case-insensitive. |
| `n_elements`    | `integer` | (Optional) The number of elements if the symbol is an array. Defaults to 1. |
| `index`         | `integer` | (Optional) The index within the PLC array that this interface corresponds to. Defaults to 0. |
| `initial_value` | `double`  | (Optional, Command Only) The initial value for a command interface before the first command is received. |
| `notify_mode`   | `string`  | (Optional, State Only) Notification transmission mode when `read_mode=notification`: `onchange` (default) pushes only when the value changes; `cyclic` pushes every `notify_cycle_ms`. Use `cyclic` for continuously-changing signals. |
| `notify_cycle_ms` | `double` | (Optional, State Only) For `onchange`, how often the PLC checks for a change; for `cyclic`, the push interval. Default 10 ms (1 ms granularity). |
| `notify_max_delay_ms` | `double` | (Optional, State Only) Server-side batching window. Default 0 = deliver each sample immediately (lowest latency). Raise to coalesce bursty signals into fewer packets. |

> Notification config is per **symbol**: one notification covers the whole symbol (all array
> elements), and the first state interface naming a symbol sets its `notify_*` values.

### Supported PLC Types
The following PLC data types are supported and are automatically converted to and from `double` values.

* `LREAL`, `REAL`
* `BOOL`
* `DINT`, `UDINT`
* `INT`, `UINT`
* `SINT`, `USINT`
* `BYTE`


## Example Project
[In this accompanying package]is an example of how to configure the hardware interface in a URDF file for a 6-axis robot and a digital output.
Here we also provide a sample PLC project with this hardware interface, along with an [example URDF](beckhoff_ads_bringup/urdf/beckhoff_bot/beckhoff_bot_macro.ros2_control.xacro)

### 1. Set up the PLC Project

First, load and compile the sample PLC project in your TwinCAT XAE environment. TwinCAT XAE is available for non-commercial use with a trial license

1.  **Import the PLC Project**: Import the from the `PLC-TestProject` directory.
2.  **Import the Library**: The PLC program requires the `tc3_interfaces` [library](PLC-TestProject/Untitled1/_Libraries/beckhoff%20automation%20gmbh/tc3_interfaces/) provided with the project. In the Solution Explorer, right-click on **References**, select **Add Library**, and add the provided `.compiled-library` file. This will allow the program to compile successfully.

### 2. Configure TwinCAT Networking

For ADS communication to work, your TwinCAT system needs a static IP and a route to the ROS 2 machine.

1.  **Set a Static IP**: Assign a static IP address to the network adapter on your Windows machine that you'll use for ADS. This IP must match the `plc_ip_address` in your URDF.
2.  **Configure Firewall**: Ensure your Windows firewall allows traffic on the ADS port (TCP 851 is the default) or is disabled on the private network for testing. Verify connectivity by pinging the Windows machine from the ROS 2 host.
3.  **Set Local AMS Net ID**: In the TwinCAT systray icon, go to **Router -> Change AMS NetID...** and set it to match the `plc_ams_net_id` in your URDF. Restart TwinCAT when prompted. This is the address of your virtual PLC device.
4.  **Add Static Route**: In your project's **SYSTEM -> Routes** tab, add a **Static** route that points to your ROS 2 machine.
    * **AMS Net ID**: `local_ams_net_id` from the URDF.
    * **IP Address**: The IP address of the machine running the ROS 2 hardware interface.

### 3. Run the PLC Program

1.  **Activate Configuration**: Click the "Activate Configuration" button in the toolbar to download the hardware setup to the runtime.
2.  **Login and Run**: Select the PLC project, click **Login** to download the program, and then click **Start** to run it. The TwinCAT icon in the systray should turn green.

### 4. Run the Hardware Interface

With the PLC running and waiting for a connection, launch the `ros2_control` system.

A successful connection will produce log output similar to this, showing the interface linking to the PLC symbols and establishing communication:

```
[controller_manager]: Loading hardware 'beckhoff_bot'
[BeckhoffADSHardwareInterface]: Exporting state interfaces...
[BeckhoffADSHardwareInterface]:     sensor 'robot_sensor/currentPos_0' | hw_states_[2] <-- MAIN.currentPos[0]
...
[BeckhoffADSHardwareInterface]: Exporting command interfaces...
[BeckhoffADSHardwareInterface]:     gpio 'robot_io/joggingEnabled' | hw_commands_[0] --> MAIN.joggingEnabled
...
[BeckhoffADSHardwareInterface]: Configuring ADS device...
[BeckhoffADSHardwareInterface]:     ADS Device configured for PLC: 192.168.122.2, Port: 851
[BeckhoffADSHardwareInterface]: Requesting Device state...
[BeckhoffADSHardwareInterface]:     Communication successful! ADS State: 5, DeviceState: 0
[BeckhoffADSHardwareInterface]: Fetching ADS handles for configured PLC variables...
[BeckhoffADSHardwareInterface]:     Handles acquired
[resource_manager]: Successful 'configure' of hardware 'beckhoff_bot'
[resource_manager]: 'activate' hardware 'beckhoff_bot'
```

You can then inspect the live values from the PLC in another terminal and compare them with those in the TwinCAT XAE:

```
ros2 topic echo /controller_manager/introspection_data/full
```

## Troubleshooting

For troubleshooting network connection with TwinCAT 3 XAE, refer to [the documentation](https://download.beckhoff.com/download/document/automation/twincat3/TwinCAT_3_ADS_INTRO_EN.pdf), especially chapters 7. and 8.

*Please note that the two following issues were encountered while running the TwinCAT 3 XAE example project on a Windows Virtual Machine.*

### TwinCAT 3 XAE Crash on Windows 11 VM 
The issue occurs when switching off `Run Mode` inside `TwinCAT 3 XAE`. `TwinCAT`crashes afterwards and reboots the VM.   
The core isolation was responsible for the bug. Isolated cores can be removed from the project's `Real time Settings`. 

### ADS Connection Failure 
The ADS connection may be [refused](https://epics.cosylab.com/documentation/adsDriver/troubleshooting/index.html#ioc-fails-to-connect-with-read-frame-failed-with-error-connection-reset-by-peer-errors) even though both the PLC and the host machine can ping each other.   
The ADS router does not seem to recognize the physical IP address of the host. The host-side **virtual IP address** should be the one used instead for the route in `TwinCAT 3 XAE` and inside the `URDF`.    
In our case for example, it corresponds to the interface `virbr0`.


# Future Plans
Feel free to contribute on any of these!

### Support for STRING Data Type
We plan to add support for reading PLC `STRING` variables. As `ros2_control` state interfaces are numeric, this would likely be exposed through a separate mechanism, such as publishing to a ROS topic, for monitoring purposes.

### Handle Custom ADS Data Structures
We may investigate adding support for reading and writing to user-defined structures (DUTs) on the PLC. This would allow for more complex data to be exchanged in a single, structured block. However, this is a complex feature and is considered a low-priority research item.

## License

This ads_vendor package was created by [B-Robotized GmbH](https://www.b-robotized.com/) and is provided under the [Apache 2.0 License](https://www.apache.org/licenses/LICENSE-2.0).

The vendored Beckhoff ADS library is subject to its own license, which can be found in [its repository](https://github.com/Beckhoff/ADS).