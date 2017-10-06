// Auto-generated. Do not edit!

// (in-package vision.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class SetTrackingColours {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.hue_low = null;
      this.hue_high = null;
      this.sat_low = null;
      this.sat_high = null;
      this.val_low = null;
      this.val_high = null;
    }
    else {
      if (initObj.hasOwnProperty('hue_low')) {
        this.hue_low = initObj.hue_low
      }
      else {
        this.hue_low = 0;
      }
      if (initObj.hasOwnProperty('hue_high')) {
        this.hue_high = initObj.hue_high
      }
      else {
        this.hue_high = 0;
      }
      if (initObj.hasOwnProperty('sat_low')) {
        this.sat_low = initObj.sat_low
      }
      else {
        this.sat_low = 0;
      }
      if (initObj.hasOwnProperty('sat_high')) {
        this.sat_high = initObj.sat_high
      }
      else {
        this.sat_high = 0;
      }
      if (initObj.hasOwnProperty('val_low')) {
        this.val_low = initObj.val_low
      }
      else {
        this.val_low = 0;
      }
      if (initObj.hasOwnProperty('val_high')) {
        this.val_high = initObj.val_high
      }
      else {
        this.val_high = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetTrackingColours
    // Serialize message field [hue_low]
    bufferOffset = _serializer.uint8(obj.hue_low, buffer, bufferOffset);
    // Serialize message field [hue_high]
    bufferOffset = _serializer.uint8(obj.hue_high, buffer, bufferOffset);
    // Serialize message field [sat_low]
    bufferOffset = _serializer.uint8(obj.sat_low, buffer, bufferOffset);
    // Serialize message field [sat_high]
    bufferOffset = _serializer.uint8(obj.sat_high, buffer, bufferOffset);
    // Serialize message field [val_low]
    bufferOffset = _serializer.uint8(obj.val_low, buffer, bufferOffset);
    // Serialize message field [val_high]
    bufferOffset = _serializer.uint8(obj.val_high, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetTrackingColours
    let len;
    let data = new SetTrackingColours(null);
    // Deserialize message field [hue_low]
    data.hue_low = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [hue_high]
    data.hue_high = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [sat_low]
    data.sat_low = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [sat_high]
    data.sat_high = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [val_low]
    data.val_low = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [val_high]
    data.val_high = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 6;
  }

  static datatype() {
    // Returns string type for a message object
    return 'vision/SetTrackingColours';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2c86efe874e740a21108c4cdc260ae8a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 hue_low
    uint8 hue_high
    uint8 sat_low
    uint8 sat_high
    uint8 val_low
    uint8 val_high
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetTrackingColours(null);
    if (msg.hue_low !== undefined) {
      resolved.hue_low = msg.hue_low;
    }
    else {
      resolved.hue_low = 0
    }

    if (msg.hue_high !== undefined) {
      resolved.hue_high = msg.hue_high;
    }
    else {
      resolved.hue_high = 0
    }

    if (msg.sat_low !== undefined) {
      resolved.sat_low = msg.sat_low;
    }
    else {
      resolved.sat_low = 0
    }

    if (msg.sat_high !== undefined) {
      resolved.sat_high = msg.sat_high;
    }
    else {
      resolved.sat_high = 0
    }

    if (msg.val_low !== undefined) {
      resolved.val_low = msg.val_low;
    }
    else {
      resolved.val_low = 0
    }

    if (msg.val_high !== undefined) {
      resolved.val_high = msg.val_high;
    }
    else {
      resolved.val_high = 0
    }

    return resolved;
    }
};

module.exports = SetTrackingColours;
