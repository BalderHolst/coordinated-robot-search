//! Internal messaging system for `botbrain` robots.

use std::collections::HashSet;

use serde::{Deserialize, Serialize};

use crate::{
    shapes::{Cone, Shape},
    RobotId,
};

/// Kinds of messages that can be sent between robots
#[cfg_attr(feature = "bin-msgs", derive(bincode::Encode, bincode::Decode))]
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum MessageKind {
    /// A message containing any shape and a diff value to update the search grid of a robot
    ShapeDiff {
        /// Where the diff should be applied
        shape: Shape,
        /// The difference in "heat"
        diff: f32,
    },

    /// A message containing a cone corresponding to he camera fov
    /// and a diff value to update the search grid of a robot
    CamDiff {
        /// The camera's view cone
        cone: Cone,
        /// The difference in "heat"
        diff: f32,
    },

    /// A message an arbitrary debug string
    Debug(String),
}

#[cfg(feature = "bin-msgs")]
fn bincode_config() -> bincode::config::Configuration {
    bincode::config::standard()
        .with_variable_int_encoding()
        .with_little_endian()
}

#[cfg(feature = "bin-msgs")]
impl MessageKind {
    /// Encode a [MessageKind] to a byte array
    pub fn encode(self) -> Result<Vec<u8>, bincode::error::EncodeError> {
        self.try_into()
    }

    /// Decode a byte array to a [MessageKind]
    pub fn decode(bytes: Vec<u8>) -> Result<Self, bincode::error::DecodeError> {
        Self::try_from(bytes)
    }
}

#[cfg(feature = "bin-msgs")]
impl TryFrom<Vec<u8>> for MessageKind {
    type Error = bincode::error::DecodeError;
    fn try_from(bytes: Vec<u8>) -> Result<Self, Self::Error> {
        let (kind, _n) = bincode::decode_from_slice(bytes.as_slice(), bincode_config())?;
        Ok(kind)
    }
}

#[cfg(feature = "bin-msgs")]
impl TryFrom<MessageKind> for Vec<u8> {
    type Error = bincode::error::EncodeError;
    fn try_from(msg_kind: MessageKind) -> Result<Self, Self::Error> {
        let bytes = bincode::encode_to_vec(msg_kind, bincode_config())?;
        Ok(bytes)
    }
}

/// A message sent between robots
#[cfg_attr(feature = "bin-msgs", derive(bincode::Encode, bincode::Decode))]
#[derive(Debug, Clone, PartialEq)]
pub struct Message {
    /// The id of the robot that sent the message
    pub sender_id: RobotId,

    /// The kind of message
    pub kind: MessageKind,
}

#[cfg(feature = "bin-msgs")]
impl Message {
    /// Encode a [Message] to a byte array
    pub fn encode(&self) -> Result<Vec<u8>, bincode::error::EncodeError> {
        bincode::encode_to_vec(self, bincode_config())
    }

    /// Decode a byte array to a [Message]
    pub fn decode(bytes: Vec<u8>) -> Result<Self, bincode::error::DecodeError> {
        bincode::decode_from_slice(bytes.as_slice(), bincode_config()).map(|(msg, _)| msg)
    }
}

#[cfg(feature = "bin-msgs")]
impl TryFrom<Vec<u8>> for Message {
    type Error = bincode::error::DecodeError;
    fn try_from(value: Vec<u8>) -> Result<Self, Self::Error> {
        Ok(bincode::decode_from_slice(value.as_slice(), bincode_config())?.0)
    }
}

#[cfg(feature = "bin-msgs")]
impl TryFrom<Message> for Vec<u8> {
    type Error = bincode::error::EncodeError;
    fn try_from(value: Message) -> Result<Self, Self::Error> {
        bincode::encode_to_vec(value, bincode_config())
    }
}

#[test]
fn test_msg_serialization() {
    let kind = MessageKind::ShapeDiff {
        shape: Shape::Circle(crate::shapes::Circle {
            center: emath::Pos2::new(-0.33, 0.66),
            radius: 10.0,
        }),
        diff: 0.5,
    };
    let kind_bytes = kind.clone().encode().unwrap();
    let kind_decoded = MessageKind::decode(kind_bytes).unwrap();
    assert_eq!(kind, kind_decoded);
    let msg = Message {
        sender_id: RobotId::new(12),
        kind,
    };
    let msg_bytes = msg.encode().unwrap();
    let msg_decoded = Message::decode(msg_bytes).unwrap();
    assert_eq!(msg, msg_decoded);
}

/// Manages incoming and outgoing messages for a robot
#[derive(Clone, Default)]
pub struct Postbox {
    incoming_msg: Vec<Message>,
    processed_msgs: HashSet<usize>,
    outgoing_msg: Vec<Message>,
}

impl Postbox {
    /// Create a new postbox
    pub fn new() -> Self {
        Self::default()
    }

    /// Get the messages from the other robots
    pub fn recv(&mut self) -> impl Iterator<Item = (usize, &Message)> {
        self.incoming_msg
            .iter()
            .enumerate()
            .filter(|(i, _msg)| !self.processed_msgs.contains(i))
    }

    /// Mark a message as processed
    pub fn set_processed(&mut self, idx: usize) {
        self.processed_msgs.insert(idx);
    }

    /// Clear the processed messages from the incoming messages list
    pub fn clean(&mut self) {
        self.incoming_msg = self
            .incoming_msg
            .iter()
            .cloned()
            .enumerate()
            .filter(|(i, _)| !self.processed_msgs.contains(i))
            .map(|(_, msg)| msg)
            .collect();
        self.processed_msgs.clear();
    }

    /// Send a message to the other robots.
    pub fn post(&mut self, msg: Message) {
        self.outgoing_msg.push(msg);
    }

    /// Deposit messages into the postbox to be received by the robot
    pub fn deposit(&mut self, msgs: impl Iterator<Item = Message>) {
        self.incoming_msg.extend(msgs);
    }

    /// Gather the messages ready to be sent to the other robots
    pub fn empty(&mut self) -> Vec<Message> {
        std::mem::take(&mut self.outgoing_msg)
    }
}
