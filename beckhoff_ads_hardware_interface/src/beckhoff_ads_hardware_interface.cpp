// Copyright (c) 2025, b-robotized
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.
//
// Author: Nikola Banovic
// Contributor: Hajar Bartakh

#include <limits>
#include <vector>
#include <cstdint>
#include <algorithm> // std::transform

#include "beckhoff_ads_hardware_interface/beckhoff_ads_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace beckhoff_ads_hardware_interface
{
    hardware_interface::CallbackReturn BeckhoffADSHardwareInterface::on_init(
        const hardware_interface::HardwareComponentParams & /*params*/)
    {
        logging_throttle_clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);

        return CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn BeckhoffADSHardwareInterface::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // Configure ADS Client Device
        if (!configure_ads_device())
        {
            RCLCPP_FATAL(getLogger(), "Failed to configure ADS device from URDF parameters.");
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Fill the ADSDataLayout vectors for read and write operations
        ads_read_layout_configure();
        ads_write_layout_configure();

        // Request handles for all symbolic PLC variable names
        RCLCPP_INFO(getLogger(), "Fetching ADS handles for configured PLC variables...");
        for (auto &layout : ads_item_layouts_read_)
        {
            try
            {
                layout.ads_handle = *(ads_device_->GetHandle(layout.plc_name_symbolic));
            }
            catch (const std::exception &ex)
            {
                RCLCPP_ERROR(getLogger(), "\tADS Exception getting handle for '%s': %s. Read operations for this variable will fail.", layout.plc_name_symbolic.c_str(), ex.what());
            }
        }
        for (auto &layout : ads_item_layouts_write_)
        {
            try
            {
                layout.ads_handle = *(ads_device_->GetHandle(layout.plc_name_symbolic));
            }
            catch (const std::exception &ex)
            {
                RCLCPP_ERROR(getLogger(), "\tADS Exception getting handle for '%s': %s. Write operations for this variable will fail.", layout.plc_name_symbolic.c_str(), ex.what());
            }
        }
        RCLCPP_INFO(getLogger(), "\tHandles acquired");

        // Pre-pack what we can for SUM read/write commands
        if (!build_sum_read_buffers())
        {
            RCLCPP_FATAL(getLogger(), "\tFailed to build ADS sum read buffer.");
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (!build_sum_write_buffers())
        {
            RCLCPP_FATAL(getLogger(), "\tFailed to build ADS sum write buffer.");
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Link command interfaces to their corresponding state interfaces
        RCLCPP_INFO(getLogger(), "Linking command interfaces to state interfaces...");
        for (auto &command_layout : ads_item_layouts_write_)
        {
            for (const auto &state_layout : ads_item_layouts_read_)
            {
                if (command_layout.plc_name_symbolic == state_layout.plc_name_symbolic)
                {
                    for (size_t k = 0; k < command_layout.num_elements; ++k)
                    {
                        // The pair is made of (command_interface_name, corresponding_state_interface_name)
                        auto pair = std::make_pair(command_layout.ros2_interfaces_.find(k)->second, state_layout.ros2_interfaces_.find(k)->second);
                        command_layout.state_command_interfaces_map_.emplace(pair);
                    }
                }
            }
        }

        return CallbackReturn::SUCCESS;
    }

    bool BeckhoffADSHardwareInterface::build_sum_read_buffers()
    {
        num_items_read_ = ads_item_layouts_read_.size();
        if (num_items_read_ == 0)
        {
            RCLCPP_INFO(getLogger(), "No items to configure for ADS Sum READ.");
            return true;
        }
        RCLCPP_INFO(getLogger(), "Building ADS sum READ buffer...");

        size_t total_error_block_size = num_items_read_ * sizeof(uint32_t);
        size_t total_data_block_size = 0;
        for (const auto &layout : ads_item_layouts_read_)
        {
            total_data_block_size += layout.plc_element_byte_size * layout.num_elements;
        }

        ads_buffer_sum_read_response_.resize(total_error_block_size + total_data_block_size);
        ads_buffer_sum_read_request_.clear();

        size_t current_data_offset = 0;
        size_t current_error_offset = 0;

        for (auto &layout : ads_item_layouts_read_)
        {
            layout.offset_in_read_response_data = total_error_block_size + current_data_offset;
            layout.offset_in_read_response_error = current_error_offset;

            ADS_ITEM_REQ_HEADER header;
            header.indexGroup = ADSIGRP_SYM_VALBYHND;
            header.indexOffset = layout.ads_handle;
            header.NumBytesData = layout.plc_element_byte_size * layout.num_elements;
            const uint8_t *ptr = reinterpret_cast<const uint8_t *>(&header);
            ads_buffer_sum_read_request_.insert(ads_buffer_sum_read_request_.end(), ptr, ptr + sizeof(ADS_ITEM_REQ_HEADER));

            // For interfaces targeting the same PLC symbol
            for (const auto &[index, interface_name] : layout.ros2_interfaces_)
            {
                // Fill the read instruction vector
                ReadInstruction read_instruction;
                read_instruction.read_buffer_offset_error_code = layout.offset_in_read_response_error;
                read_instruction.read_buffer_offset_data = layout.offset_in_read_response_data + index * layout.plc_element_byte_size;
                read_instruction.plc_type = layout.plc_type;
                read_instruction.state_interface_name = interface_name;

                // The state interfaces' names are ordered by ascending indexes of the PLC array thanks to layout.ros2_interfaces_ being a map
                ads_read_instructions_.push_back(read_instruction);
            }

            current_data_offset += header.NumBytesData;
            current_error_offset += sizeof(uint32_t);
        }
        RCLCPP_INFO(getLogger(), "ADS Sum READ configured for %zu items. Request: %zu bytes, Response: %zu bytes.",
                    num_items_read_, ads_buffer_sum_read_request_.size(), ads_buffer_sum_read_response_.size());
        return true;
    }

    bool BeckhoffADSHardwareInterface::build_sum_write_buffers()
    {
        RCLCPP_INFO(getLogger(), "Building ADS sum WRITE buffer...");
        num_items_write_ = ads_item_layouts_write_.size();
        if (num_items_write_ == 0)
        {
            RCLCPP_INFO(getLogger(), "No items to configure for ADS Sum WRITE.");
            return true;
        }

        size_t total_data_size = 0;
        size_t total_header_size = num_items_write_ * sizeof(ADS_ITEM_REQ_HEADER);
        for (const auto &layout : ads_item_layouts_write_)
        {
            total_data_size += layout.plc_element_byte_size * layout.num_elements;
        }

        ads_buffer_sum_write_request_.resize(total_header_size + total_data_size);
        ads_buffer_sum_write_response_.resize(num_items_write_ * sizeof(uint32_t));

        auto *header_block_ptr = reinterpret_cast<ADS_ITEM_REQ_HEADER *>(ads_buffer_sum_write_request_.data());
        size_t current_data_offset = 0;
        size_t i = 0; // Index for the header block pointer

        for (auto &layout : ads_item_layouts_write_)
        {
            header_block_ptr[i].indexGroup = ADSIGRP_SYM_VALBYHND;
            header_block_ptr[i].indexOffset = layout.ads_handle;
            header_block_ptr[i].NumBytesData = layout.plc_element_byte_size * layout.num_elements;

            layout.offset_in_write_request_data = total_header_size + current_data_offset;

            // For interfaces targeting the same PLC symbol
            for (const auto &[index, interface_name] : layout.ros2_interfaces_)
            {
                // Fill the write instruction vector
                WriteInstruction write_instruction;
                write_instruction.write_buffer_offset_data = layout.offset_in_write_request_data + index * layout.plc_element_byte_size;
                write_instruction.plc_type = layout.plc_type;
                write_instruction.command_interface_name = interface_name;
                write_instruction.fallback_state_interface_name = "";

                // There exists a state interface for the same PLC symbol
                if (!layout.state_command_interfaces_map_.empty())
                {
                    write_instruction.fallback_state_interface_name = layout.state_command_interfaces_map_.find(interface_name)->second;
                }

                // The command interfaces' names are ordered by ascending indexes of the PLC array thanks to layout.ros2_interfaces_ being a map
                ads_write_instructions_.push_back(write_instruction);
            }

            current_data_offset += header_block_ptr[i].NumBytesData;
            i++;
        }

        RCLCPP_INFO(getLogger(), "ADS Sum WRITE configured for %zu items. Request: %zu bytes, Response: %zu bytes.",
                    num_items_write_, ads_buffer_sum_write_request_.size(), ads_buffer_sum_write_response_.size());
        return true;
    }

    void BeckhoffADSHardwareInterface::ads_read_layout_configure()
    {
        // Count all state interfaces to pre-allocate memory once and avoid reallocations.
        size_t num_state_interfaces = gpio_state_interfaces_.size() + joint_state_interfaces_.size() + sensor_state_interfaces_.size();

        // Reserve worst-case scenario for layouts (each interface targets a different PLC symbol)
        ads_item_layouts_read_.clear();
        ads_item_layouts_read_.reserve(num_state_interfaces);

        // Keep track of multiple interfaces targeting the same PLC symbol of type ARRAY[x], but different index
        std::map<std::string, bool> processed_plc_symbols;

        auto init_ads_read_layout =
            [&](const auto &type_state_interfaces_)
        {
            for (const auto &[name, descr] : type_state_interfaces_)
            {
                std::string plc_symbol;
                std::string plc_type_str;
                size_t num_elements = 1;
                size_t plc_index = 0;
                try
                {
                    plc_symbol = descr.interface_info.parameters.at("PLC_symbol");
                    plc_type_str = descr.interface_info.parameters.at("PLC_type");
                    if (descr.interface_info.parameters.count("n_elements"))
                    {
                        num_elements = std::stoul(descr.interface_info.parameters.at("n_elements"));
                    }
                    if (descr.interface_info.parameters.count("index"))
                    {
                        plc_index = std::stoul(descr.interface_info.parameters.at("index"));
                    }
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR(getLogger(), "Error parsing PLC parameters for state interface '%s': %s. Check URDF.",
                                 name.c_str(), e.what());
                    continue;
                }

                // If this is the first time we see this symbol, create the layout
                if (processed_plc_symbols.find(plc_symbol) == processed_plc_symbols.end())
                {
                    ADSDataLayout layout;
                    layout.plc_name_symbolic = plc_symbol;
                    layout.num_elements = num_elements;
                    layout.plc_type = strToPlcType(plc_type_str);
                    layout.ros2_interfaces_.emplace(std::make_pair(plc_index, name));

                    if (layout.plc_type == PLCType::UNKNOWN || layout.plc_type == PLCType::STRING)
                    {
                        RCLCPP_ERROR(getLogger(), "Skipping state variable '%s' due to UNSUPPORTED or UNKNOWN PLC_type '%s'.", layout.plc_name_symbolic.c_str(), plc_type_str.c_str());
                    }
                    else
                    {
                        layout.plc_element_byte_size = plcTypeByteSize(layout.plc_type);
                        ads_item_layouts_read_.push_back(layout);
                        processed_plc_symbols[plc_symbol] = true;
                    }
                }
                // The symbol already exists
                else
                {
                    // Find the ADS Data Layout object of the corresponding PLC symbol
                    auto it = std::find_if(ads_item_layouts_read_.begin(), ads_item_layouts_read_.end(),
                                           [&plc_symbol](ADSDataLayout layout)
                                           { return layout.plc_name_symbolic == plc_symbol; });

                    // Add the interface name the layout
                    (*it).ros2_interfaces_.emplace(std::make_pair(plc_index, name));
                }
            }
        };

        init_ads_read_layout(joint_state_interfaces_);
        init_ads_read_layout(gpio_state_interfaces_);
        init_ads_read_layout(sensor_state_interfaces_);
    }

    void BeckhoffADSHardwareInterface::ads_write_layout_configure()
    {
        // Count all command interfaces to pre-allocate memory once and avoid reallocations.
        size_t num_command_interfaces = joint_command_interfaces_.size() + gpio_command_interfaces_.size();

        // Reserve worst-case scenario for layouts (each interface targets a different PLC symbol)
        ads_item_layouts_write_.clear();
        ads_item_layouts_write_.reserve(num_command_interfaces);

        // Keep track of multiple interfaces targeting the same PLC symbol of type ARRAY[x], but different index
        std::map<std::string, bool> processed_plc_symbols;

        auto init_ads_write_layout =
            [&](const auto &type_command_interfaces_)
        {
            for (const auto &[name, descr] : type_command_interfaces_)
            {
                [[maybe_unused]] double initial_value = std::numeric_limits<double>::quiet_NaN();

                if (descr.interface_info.parameters.count("initial_value"))
                {
                    try
                    {
                        initial_value = std::stod(descr.interface_info.parameters.at("initial_value"));
                    }
                    catch (const std::exception &ex)
                    { // Catch conversion errors
                        RCLCPP_WARN(
                            getLogger(),
                            "Invalid 'initial_value' ('%s') for command interface '%s'. Using NaN. Error: %s",
                            descr.interface_info.parameters.at("initial_value").c_str(),
                            name.c_str(),
                            ex.what());
                    }
                }

                std::string plc_symbol;
                std::string plc_type_str;
                size_t num_elements = 1;
                size_t plc_index = 0;
                try
                {
                    plc_symbol = descr.interface_info.parameters.at("PLC_symbol");
                    plc_type_str = descr.interface_info.parameters.at("PLC_type");
                    if (descr.interface_info.parameters.count("n_elements"))
                    {
                        num_elements = std::stoul(descr.interface_info.parameters.at("n_elements"));
                    }
                    if (descr.interface_info.parameters.count("index"))
                    {
                        plc_index = std::stoul(descr.interface_info.parameters.at("index"));
                    }
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR(getLogger(), "Error parsing PLC parameters for command interface '%s': %s. Check URDF.",
                                 name.c_str(), e.what());
                    continue;
                }

                if (processed_plc_symbols.find(plc_symbol) == processed_plc_symbols.end())
                {
                    ADSDataLayout layout;
                    layout.plc_name_symbolic = plc_symbol;
                    layout.num_elements = num_elements;
                    layout.plc_type = strToPlcType(plc_type_str);
                    layout.ros2_interfaces_.emplace(std::make_pair(plc_index, name));

                    if (layout.plc_type == PLCType::UNKNOWN || layout.plc_type == PLCType::STRING)
                    {
                        RCLCPP_ERROR(getLogger(), "Skipping command variable '%s' due to UNSUPPORTED or UNKNOWN PLC_type '%s'.", layout.plc_name_symbolic.c_str(), plc_type_str.c_str());
                    }
                    else
                    {
                        layout.plc_element_byte_size = plcTypeByteSize(layout.plc_type);
                        ads_item_layouts_write_.push_back(layout);
                        processed_plc_symbols[plc_symbol] = true;
                    }
                }
                // The symbol already exists
                else
                {
                    // Look for the ADS Data Layout of the corresponding PLC symbol
                    auto it = std::find_if(ads_item_layouts_write_.begin(), ads_item_layouts_write_.end(),
                                           [&plc_symbol](ADSDataLayout layout)
                                           { return layout.plc_name_symbolic == plc_symbol; });

                    // Add the command interface name the layout
                    (*it).ros2_interfaces_.emplace(std::make_pair(plc_index, name));
                }
            }
        };

        init_ads_write_layout(joint_command_interfaces_);
        init_ads_write_layout(gpio_command_interfaces_);
    }

    hardware_interface::CallbackReturn BeckhoffADSHardwareInterface::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn BeckhoffADSHardwareInterface::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // TODO: send some safety commands to the PLC?
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type BeckhoffADSHardwareInterface::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        if (num_items_read_ == 0)
        {
            return hardware_interface::return_type::OK;
        }

        uint32_t bytes_read_from_plc = 0;
        long ads_sum_read_error = ads_device_->ReadWriteReqEx2(
            ADSIGRP_SUMUP_READ,
            num_items_read_,
            ads_buffer_sum_read_response_.size(),
            ads_buffer_sum_read_response_.data(),
            ads_buffer_sum_read_request_.size(),
            ads_buffer_sum_read_request_.data(),
            &bytes_read_from_plc);

        if (ads_sum_read_error != ADSERR_NOERR)
        {
            RCLCPP_ERROR_THROTTLE(getLogger(), *logging_throttle_clock_, 1000,
                                  "Overall ADS Sum Read Error: 0x%lX.", ads_sum_read_error);
            // TODO: See if we need to do something on read error. Maybe assign NaN to all interfaces?
            return hardware_interface::return_type::ERROR;
        }

        if (bytes_read_from_plc != ads_buffer_sum_read_response_.size())
        {
            RCLCPP_ERROR_THROTTLE(getLogger(), *logging_throttle_clock_, 1000,
                                  "ADS Sum Read size mismatch. Expected %zu, Got %u.",
                                  ads_buffer_sum_read_response_.size(), bytes_read_from_plc);
            return hardware_interface::return_type::ERROR;
        }

        bool any_item_read_failed = false;
        for (const auto &read_instruction : ads_read_instructions_)
        {
            uint32_t item_error_code;
            memcpy(&item_error_code,
                   ads_buffer_sum_read_response_.data() + read_instruction.read_buffer_offset_error_code,
                   sizeof(uint32_t));

            if (item_error_code != ADSERR_NOERR)
            {
                RCLCPP_WARN_THROTTLE(getLogger(), *logging_throttle_clock_, 1000,
                                     "ADS Sum Read operation corresponding to the state interface '%s' failed: 0x%X.",
                                     read_instruction.state_interface_name.c_str(), item_error_code);

                // TODO: See if we need to do something on read error. Maybe assign NaN to the interface?
                any_item_read_failed = true;
                continue;
            }

            // Each state interface has its corresponding read_instruction
            const uint8_t *ptr_plc_element_current = ads_buffer_sum_read_response_.data() + read_instruction.read_buffer_offset_data;

            // TODO: performance - Hoist the switch/case above for loop?

            switch (read_instruction.plc_type)
            {
            case PLCType::LREAL:
            {
                double val;
                memcpy(&val, ptr_plc_element_current, plcTypeByteSize(read_instruction.plc_type));
                set_state(read_instruction.state_interface_name, val);
                break;
            }
            case PLCType::REAL:
            {
                float val;
                memcpy(&val, ptr_plc_element_current, plcTypeByteSize(read_instruction.plc_type));
                set_state(read_instruction.state_interface_name, static_cast<double>(val));
                break;
            }
            case PLCType::BOOL:
            {
                uint8_t byte_val;
                memcpy(&byte_val, ptr_plc_element_current, plcTypeByteSize(read_instruction.plc_type));
                set_state(read_instruction.state_interface_name, (byte_val != 0) ? 1.0 : 0.0);
                break;
            }
            case PLCType::SINT:
            {
                int8_t val;
                memcpy(&val, ptr_plc_element_current, plcTypeByteSize(read_instruction.plc_type));
                set_state(read_instruction.state_interface_name, static_cast<double>(val));
                break;
            }
            case PLCType::USINT:
            case PLCType::BYTE:
            {
                uint8_t val;
                memcpy(&val, ptr_plc_element_current, plcTypeByteSize(read_instruction.plc_type));
                set_state(read_instruction.state_interface_name, static_cast<double>(val));
                break;
            }
            case PLCType::INT:
            {
                int16_t val;
                memcpy(&val, ptr_plc_element_current, plcTypeByteSize(read_instruction.plc_type));
                set_state(read_instruction.state_interface_name, static_cast<double>(val));
                break;
            }
            case PLCType::UINT:
            {
                uint16_t val;
                memcpy(&val, ptr_plc_element_current, plcTypeByteSize(read_instruction.plc_type));
                set_state(read_instruction.state_interface_name, static_cast<double>(val));
                break;
            }
            case PLCType::DINT:
            {
                int32_t val;
                memcpy(&val, ptr_plc_element_current, plcTypeByteSize(read_instruction.plc_type));
                set_state(read_instruction.state_interface_name, static_cast<double>(val));
                break;
            }
            case PLCType::UDINT:
            {
                uint32_t val;
                memcpy(&val, ptr_plc_element_current, plcTypeByteSize(read_instruction.plc_type));
                set_state(read_instruction.state_interface_name, static_cast<double>(val));
                break;
            }
            /* Not supported for now, guarded against in on_configure()
            case PLCType::STRING:
                break;
            */
            case PLCType::UNKNOWN:
            default:
                RCLCPP_ERROR_THROTTLE(getLogger(), *logging_throttle_clock_, 1000,
                                      "Unhandled or UNKNOWN PLC type (%d) for the interface '%s' during read.",
                                      static_cast<int>(read_instruction.plc_type), read_instruction.state_interface_name.c_str());
                set_state(read_instruction.state_interface_name, std::numeric_limits<double>::quiet_NaN());
                any_item_read_failed = true;
                break;
            }
        }
        return any_item_read_failed ? hardware_interface::return_type::ERROR : hardware_interface::return_type::OK;
    }

    hardware_interface::return_type BeckhoffADSHardwareInterface::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        if (num_items_write_ == 0)
        {
            return hardware_interface::return_type::OK;
        }

        for (const auto &write_instruction : ads_write_instructions_)
        {
            uint8_t *ptr_write_buffer_destination_current = ads_buffer_sum_write_request_.data() + write_instruction.write_buffer_offset_data;

            // TODO: performance - Hoist the switch/case above for loop?

            // store the current val and reset the ros-side command value
            double val = get_command(write_instruction.command_interface_name);
            set_command(write_instruction.command_interface_name, std::numeric_limits<double>::quiet_NaN());

            if (std::isnan(val))
            {
                // if the original value was NaN and there exist a state interface of the same name, write corresponding state interface
                if (!write_instruction.fallback_state_interface_name.empty())
                {
                    val = get_state(write_instruction.fallback_state_interface_name);
                }

                // if we STILL don't have a fallback value on, don't update the write buffer.
                // the last valid command is written
                if (std::isnan(val))
                {
                    continue;
                }
            }

            switch (write_instruction.plc_type)
            {
            case PLCType::LREAL:
            {
                // val is already double (LREAL is 8 bytes - 64 bit)
                memcpy(ptr_write_buffer_destination_current, &val, plcTypeByteSize(write_instruction.plc_type));
                break;
            }
            case PLCType::REAL:
            {
                float plc_val = static_cast<float>(val);
                memcpy(ptr_write_buffer_destination_current, &plc_val, plcTypeByteSize(write_instruction.plc_type));
                break;
            }
            case PLCType::BOOL:
            {
                // bool is size of byte in PLC
                uint8_t plc_val = (val != 0.0) ? 1 : 0;
                memcpy(ptr_write_buffer_destination_current, &plc_val, plcTypeByteSize(write_instruction.plc_type));
                break;
            }
            case PLCType::SINT:
            {
                int8_t plc_val = static_cast<int8_t>(std::round(val));
                memcpy(ptr_write_buffer_destination_current, &plc_val, plcTypeByteSize(write_instruction.plc_type));
                break;
            }
            case PLCType::USINT:
            case PLCType::BYTE:
            {
                uint8_t plc_val = static_cast<uint8_t>(std::round(val));
                memcpy(ptr_write_buffer_destination_current, &plc_val, plcTypeByteSize(write_instruction.plc_type));
                break;
            }
            case PLCType::INT:
            {
                int16_t plc_val = static_cast<int16_t>(std::round(val));
                memcpy(ptr_write_buffer_destination_current, &plc_val, plcTypeByteSize(write_instruction.plc_type));
                break;
            }
            case PLCType::UINT:
            {
                uint16_t plc_val = static_cast<uint16_t>(std::round(val));
                memcpy(ptr_write_buffer_destination_current, &plc_val, plcTypeByteSize(write_instruction.plc_type));
                break;
            }
            case PLCType::DINT:
            {
                int32_t plc_val = static_cast<int32_t>(std::round(val));
                memcpy(ptr_write_buffer_destination_current, &plc_val, plcTypeByteSize(write_instruction.plc_type));
                break;
            }
            case PLCType::UDINT:
            {
                uint32_t plc_val = static_cast<uint32_t>(std::round(val));
                memcpy(ptr_write_buffer_destination_current, &plc_val, plcTypeByteSize(write_instruction.plc_type));
                break;
            }
            /* String not supported for now
            case PLCType::STRING: break;
            */
            case PLCType::UNKNOWN:
            default:
                RCLCPP_FATAL(getLogger(), "UNKNOWN PLC type (%d) for the interface '%s' during write. Sending zeroed data of size %zu.",
                             static_cast<int>(write_instruction.plc_type), write_instruction.command_interface_name.c_str(), plcTypeByteSize(write_instruction.plc_type));
                return hardware_interface::return_type::ERROR;
                break;
            }
        }

        uint32_t bytes_response_buffer_from_plc = 0;
        long ads_sum_write_error = ads_device_->ReadWriteReqEx2(
            ADSIGRP_SUMUP_WRITE,
            num_items_write_,
            ads_buffer_sum_write_response_.size(),
            ads_buffer_sum_write_response_.data(),
            ads_buffer_sum_write_request_.size(),
            ads_buffer_sum_write_request_.data(),
            &bytes_response_buffer_from_plc);

        if (ads_sum_write_error != ADSERR_NOERR)
        {
            RCLCPP_ERROR_THROTTLE(getLogger(), *logging_throttle_clock_, 1000,
                                  "Overall ADS Sum Write Error: 0x%lX.", ads_sum_write_error);
            return hardware_interface::return_type::ERROR;
        }

        if (bytes_response_buffer_from_plc != ads_buffer_sum_write_response_.size())
        {
            RCLCPP_ERROR_THROTTLE(getLogger(), *logging_throttle_clock_, 1000,
                                  "ADS Sum Write response size mismatch (error codes). Expected %zu, Got %u.",
                                  ads_buffer_sum_write_response_.size(), bytes_response_buffer_from_plc);
        }

        bool any_item_write_failed = false;
        for (size_t i = 0; i < num_items_write_; ++i)
        {
            uint32_t item_error_code;
            memcpy(&item_error_code, ads_buffer_sum_write_response_.data() + i * sizeof(uint32_t), sizeof(uint32_t));
            if (item_error_code != ADSERR_NOERR)
            {
                RCLCPP_WARN_THROTTLE(getLogger(), *logging_throttle_clock_, 1000,
                                     "ADS Sum Write sub-op for '%s' (handle 0x%X) failed: 0x%X",
                                     ads_item_layouts_write_[i].plc_name_symbolic.c_str(), ads_item_layouts_write_[i].ads_handle, item_error_code);
                any_item_write_failed = true;
            }
        }

        return any_item_write_failed ? hardware_interface::return_type::ERROR : hardware_interface::return_type::OK;
    }

    hardware_interface::CallbackReturn BeckhoffADSHardwareInterface::on_shutdown(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(getLogger(), "Releasing ADS resources...");
        if (ads_device_)
        {
            ads_device_.reset();
        }
        RCLCPP_INFO(getLogger(), "ADS resources released.");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    bool BeckhoffADSHardwareInterface::configure_ads_device()
    {
        RCLCPP_INFO(getLogger(), "Configuring ADS device...");
        const auto &params = info_.hardware_parameters;
        try
        {
            std::string plc_ip = params.at("plc_ip_address");
            std::string plc_ams_net_id_str = params.at("plc_ams_net_id");
            std::string local_ams_net_id_str = params.at("local_ams_net_id");
            uint16_t plc_ams_port = std::stoul(params.at("plc_ams_port"));

            AmsNetId remote_net_id;
            if (sscanf(plc_ams_net_id_str.c_str(), "%hhu.%hhu.%hhu.%hhu.%hhu.%hhu",
                       &remote_net_id.b[0], &remote_net_id.b[1], &remote_net_id.b[2],
                       &remote_net_id.b[3], &remote_net_id.b[4], &remote_net_id.b[5]) != 6)
            {
                RCLCPP_FATAL(getLogger(), "\tInvalid format for 'plc_ams_net_id'. Expected 'x.x.x.x.x.x'.");
                return false;
            }

            AmsNetId local_net_id;
            if (sscanf(local_ams_net_id_str.c_str(), "%hhu.%hhu.%hhu.%hhu.%hhu.%hhu",
                       &local_net_id.b[0], &local_net_id.b[1], &local_net_id.b[2],
                       &local_net_id.b[3], &local_net_id.b[4], &local_net_id.b[5]) != 6)
            {
                RCLCPP_FATAL(getLogger(), "\tInvalid format for 'local_ams_net_id'. Expected 'x.x.x.x.x.x'.");
                return false;
            }

            bhf::ads::SetLocalAddress(local_net_id);
            ads_device_ = std::make_unique<AdsDevice>(plc_ip, remote_net_id, plc_ams_port);
            RCLCPP_INFO(getLogger(), "\tTimeout is: %u", ads_device_->GetTimeout());

            RCLCPP_INFO(getLogger(), "\tADS Device configured for PLC: %s, Port: %u", plc_ip.c_str(), plc_ams_port);
            RCLCPP_INFO(getLogger(), "\tPLC AMS NetID: %s", plc_ams_net_id_str.c_str());

            RCLCPP_INFO(getLogger(), "Requesting Device state...");
            AdsDeviceState deviceState = ads_device_->GetState();
            RCLCPP_INFO(getLogger(), "\tCommunication successful! ADS State: %d, DeviceState: %d", deviceState.ads, deviceState.device);
        }
        catch (const std::out_of_range &ex)
        {
            RCLCPP_FATAL(getLogger(), "\tMissing required URDF <hardware> parameter: %s", ex.what());
            return false;
        }
        catch (const AdsException &ex)
        {
            RCLCPP_FATAL(getLogger(), "\tADS Exception during connection: %s (Error Code: 0x%lX)", ex.what(), ex.errorCode);
            return false;
        }
        catch (const std::exception &ex)
        {
            RCLCPP_FATAL(getLogger(), "\tError during ADS connection config: %s", ex.what());
            return false;
        }

        return true;
    }

    PLCType BeckhoffADSHardwareInterface::strToPlcType(const std::string &type_str_param)
    {
        std::string type_str = type_str_param;
        std::transform(type_str.begin(), type_str.end(), type_str.begin(), ::toupper);

        if (type_str == "LREAL")
            return PLCType::LREAL;
        if (type_str == "REAL")
            return PLCType::REAL;
        if (type_str == "BOOL")
            return PLCType::BOOL;
        if (type_str == "UDINT")
            return PLCType::UDINT;
        if (type_str == "DINT")
            return PLCType::DINT;
        if (type_str == "UINT")
            return PLCType::UINT;
        if (type_str == "INT")
            return PLCType::INT;
        if (type_str == "USINT")
            return PLCType::USINT;
        if (type_str == "SINT")
            return PLCType::SINT;
        if (type_str == "BYTE")
            return PLCType::BYTE;
        if (type_str == "STRING")
            return PLCType::STRING;

        RCLCPP_ERROR(getLogger(), "Unknown PLC type string: '%s'", type_str_param.c_str());
        return PLCType::UNKNOWN;
    }

    size_t BeckhoffADSHardwareInterface::plcTypeByteSize(PLCType plc_type_enum)
    {
        switch (plc_type_enum)
        {
        case PLCType::LREAL:
            return 8;
        case PLCType::REAL:
            return 4;
        case PLCType::BOOL:
            return 1;
        case PLCType::UDINT:
            return 4;
        case PLCType::DINT:
            return 4;
        case PLCType::UINT:
            return 2;
        case PLCType::INT:
            return 2;
        case PLCType::USINT:
            return 1;
        case PLCType::SINT:
            return 1;
        case PLCType::BYTE:
            return 1;
        // case PLCType::STRING: not currently supported
        case PLCType::UNKNOWN:
        default:
            RCLCPP_ERROR(getLogger(), "Cannot get byte size for UNKNOWN or unhandled PLC type enum value: %d", static_cast<int>(plc_type_enum));
            return 0;
        }
    }

} // namespace beckhoff_ads_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    beckhoff_ads_hardware_interface::BeckhoffADSHardwareInterface, hardware_interface::SystemInterface)