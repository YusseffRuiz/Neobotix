// Auto-generated. Do not edit!

// (in-package neo_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class IOBoard {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.digital_inputs = null;
      this.digital_outputs = null;
      this.analog_inputs = null;
    }
    else {
      if (initObj.hasOwnProperty('digital_inputs')) {
        this.digital_inputs = initObj.digital_inputs
      }
      else {
        this.digital_inputs = new Array(16).fill(0);
      }
      if (initObj.hasOwnProperty('digital_outputs')) {
        this.digital_outputs = initObj.digital_outputs
      }
      else {
        this.digital_outputs = new Array(16).fill(0);
      }
      if (initObj.hasOwnProperty('analog_inputs')) {
        this.analog_inputs = initObj.analog_inputs
      }
      else {
        this.analog_inputs = new Array(8).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type IOBoard
    // Check that the constant length array field [digital_inputs] has the right length
    if (obj.digital_inputs.length !== 16) {
      throw new Error('Unable to serialize array field digital_inputs - length must be 16')
    }
    // Serialize message field [digital_inputs]
    bufferOffset = _arraySerializer.bool(obj.digital_inputs, buffer, bufferOffset, 16);
    // Check that the constant length array field [digital_outputs] has the right length
    if (obj.digital_outputs.length !== 16) {
      throw new Error('Unable to serialize array field digital_outputs - length must be 16')
    }
    // Serialize message field [digital_outputs]
    bufferOffset = _arraySerializer.bool(obj.digital_outputs, buffer, bufferOffset, 16);
    // Check that the constant length array field [analog_inputs] has the right length
    if (obj.analog_inputs.length !== 8) {
      throw new Error('Unable to serialize array field analog_inputs - length must be 8')
    }
    // Serialize message field [analog_inputs]
    bufferOffset = _arraySerializer.uint8(obj.analog_inputs, buffer, bufferOffset, 8);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type IOBoard
    let len;
    let data = new IOBoard(null);
    // Deserialize message field [digital_inputs]
    data.digital_inputs = _arrayDeserializer.bool(buffer, bufferOffset, 16)
    // Deserialize message field [digital_outputs]
    data.digital_outputs = _arrayDeserializer.bool(buffer, bufferOffset, 16)
    // Deserialize message field [analog_inputs]
    data.analog_inputs = _arrayDeserializer.uint8(buffer, bufferOffset, 8)
    return data;
  }

  static getMessageSize(object) {
    return 40;
  }

  static datatype() {
    // Returns string type for a message object
    return 'neo_msgs/IOBoard';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bfa16d6202dd05ce5b3fdebf8a7d44db';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # This message holds all data available from Neobotix IOBoard
     
    bool[16] digital_inputs			# state for all digital inputs
    bool[16] digital_outputs		# state for all digital outputs
    
    uint8[8] analog_inputs			# analog input values
    
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new IOBoard(null);
    if (msg.digital_inputs !== undefined) {
      resolved.digital_inputs = msg.digital_inputs;
    }
    else {
      resolved.digital_inputs = new Array(16).fill(0)
    }

    if (msg.digital_outputs !== undefined) {
      resolved.digital_outputs = msg.digital_outputs;
    }
    else {
      resolved.digital_outputs = new Array(16).fill(0)
    }

    if (msg.analog_inputs !== undefined) {
      resolved.analog_inputs = msg.analog_inputs;
    }
    else {
      resolved.analog_inputs = new Array(8).fill(0)
    }

    return resolved;
    }
};

module.exports = IOBoard;
