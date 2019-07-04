// Auto-generated. Do not edit!

// (in-package cob_srvs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetOperationModeRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.operation_mode = null;
    }
    else {
      if (initObj.hasOwnProperty('operation_mode')) {
        this.operation_mode = initObj.operation_mode
      }
      else {
        this.operation_mode = new std_msgs.msg.String();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetOperationModeRequest
    // Serialize message field [operation_mode]
    bufferOffset = std_msgs.msg.String.serialize(obj.operation_mode, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetOperationModeRequest
    let len;
    let data = new SetOperationModeRequest(null);
    // Deserialize message field [operation_mode]
    data.operation_mode = std_msgs.msg.String.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.String.getMessageSize(object.operation_mode);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'cob_srvs/SetOperationModeRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'df954cef597fce5b749fee53c7846c40';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/String operation_mode
    
    ================================================================================
    MSG: std_msgs/String
    string data
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetOperationModeRequest(null);
    if (msg.operation_mode !== undefined) {
      resolved.operation_mode = std_msgs.msg.String.Resolve(msg.operation_mode)
    }
    else {
      resolved.operation_mode = new std_msgs.msg.String()
    }

    return resolved;
    }
};

class SetOperationModeResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.error_message = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = new std_msgs.msg.Bool();
      }
      if (initObj.hasOwnProperty('error_message')) {
        this.error_message = initObj.error_message
      }
      else {
        this.error_message = new std_msgs.msg.String();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetOperationModeResponse
    // Serialize message field [success]
    bufferOffset = std_msgs.msg.Bool.serialize(obj.success, buffer, bufferOffset);
    // Serialize message field [error_message]
    bufferOffset = std_msgs.msg.String.serialize(obj.error_message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetOperationModeResponse
    let len;
    let data = new SetOperationModeResponse(null);
    // Deserialize message field [success]
    data.success = std_msgs.msg.Bool.deserialize(buffer, bufferOffset);
    // Deserialize message field [error_message]
    data.error_message = std_msgs.msg.String.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.String.getMessageSize(object.error_message);
    return length + 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'cob_srvs/SetOperationModeResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bdad515e8f4c013bd650d3407753d2b8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Bool success
    std_msgs/String error_message
    
    
    ================================================================================
    MSG: std_msgs/Bool
    bool data
    ================================================================================
    MSG: std_msgs/String
    string data
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetOperationModeResponse(null);
    if (msg.success !== undefined) {
      resolved.success = std_msgs.msg.Bool.Resolve(msg.success)
    }
    else {
      resolved.success = new std_msgs.msg.Bool()
    }

    if (msg.error_message !== undefined) {
      resolved.error_message = std_msgs.msg.String.Resolve(msg.error_message)
    }
    else {
      resolved.error_message = new std_msgs.msg.String()
    }

    return resolved;
    }
};

module.exports = {
  Request: SetOperationModeRequest,
  Response: SetOperationModeResponse,
  md5sum() { return '588a6152f0ed57d725118cc82be8479a'; },
  datatype() { return 'cob_srvs/SetOperationMode'; }
};
