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

class kalmanLocalizationPose {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.stateMean = null;
      this.stateCovariance = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('stateMean')) {
        this.stateMean = initObj.stateMean
      }
      else {
        this.stateMean = [];
      }
      if (initObj.hasOwnProperty('stateCovariance')) {
        this.stateCovariance = initObj.stateCovariance
      }
      else {
        this.stateCovariance = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type kalmanLocalizationPose
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [stateMean]
    bufferOffset = _arraySerializer.float32(obj.stateMean, buffer, bufferOffset, null);
    // Serialize message field [stateCovariance]
    bufferOffset = _arraySerializer.float32(obj.stateCovariance, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type kalmanLocalizationPose
    let len;
    let data = new kalmanLocalizationPose(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [stateMean]
    data.stateMean = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [stateCovariance]
    data.stateCovariance = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 4 * object.stateMean.length;
    length += 4 * object.stateCovariance.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rvss_workshop/kalmanLocalizationPose';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3241c9839fd6056d6120443c4c3f5013';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    float32[] stateMean
    float32[] stateCovariance
    
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
    const resolved = new kalmanLocalizationPose(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.stateMean !== undefined) {
      resolved.stateMean = msg.stateMean;
    }
    else {
      resolved.stateMean = []
    }

    if (msg.stateCovariance !== undefined) {
      resolved.stateCovariance = msg.stateCovariance;
    }
    else {
      resolved.stateCovariance = []
    }

    return resolved;
    }
};

module.exports = kalmanLocalizationPose;
