// Auto-generated. Do not edit!

// (in-package rvss_workshop.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class objMsg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.Zrobot = null;
      this.Xrobot = null;
      this.label = null;
      this.covariance = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('Zrobot')) {
        this.Zrobot = initObj.Zrobot
      }
      else {
        this.Zrobot = 0.0;
      }
      if (initObj.hasOwnProperty('Xrobot')) {
        this.Xrobot = initObj.Xrobot
      }
      else {
        this.Xrobot = 0.0;
      }
      if (initObj.hasOwnProperty('label')) {
        this.label = initObj.label
      }
      else {
        this.label = 0;
      }
      if (initObj.hasOwnProperty('covariance')) {
        this.covariance = initObj.covariance
      }
      else {
        this.covariance = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type objMsg
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [Zrobot]
    bufferOffset = _serializer.float32(obj.Zrobot, buffer, bufferOffset);
    // Serialize message field [Xrobot]
    bufferOffset = _serializer.float32(obj.Xrobot, buffer, bufferOffset);
    // Serialize message field [label]
    bufferOffset = _serializer.uint8(obj.label, buffer, bufferOffset);
    // Serialize message field [covariance]
    bufferOffset = _arraySerializer.float32(obj.covariance, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type objMsg
    let len;
    let data = new objMsg(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [Zrobot]
    data.Zrobot = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [Xrobot]
    data.Xrobot = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [label]
    data.label = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [covariance]
    data.covariance = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 4 * object.covariance.length;
    return length + 13;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rvss_workshop/objMsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '92039726cacaf0739b94657d27987f60';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    float32 Zrobot
    float32 Xrobot
    uint8 label
    float32[] covariance
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    # 0: no frame
    # 1: global frame
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new objMsg(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.Zrobot !== undefined) {
      resolved.Zrobot = msg.Zrobot;
    }
    else {
      resolved.Zrobot = 0.0
    }

    if (msg.Xrobot !== undefined) {
      resolved.Xrobot = msg.Xrobot;
    }
    else {
      resolved.Xrobot = 0.0
    }

    if (msg.label !== undefined) {
      resolved.label = msg.label;
    }
    else {
      resolved.label = 0
    }

    if (msg.covariance !== undefined) {
      resolved.covariance = msg.covariance;
    }
    else {
      resolved.covariance = []
    }

    return resolved;
    }
};

module.exports = objMsg;
