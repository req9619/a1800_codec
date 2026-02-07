pub mod analysis;
pub mod bitstream;
pub mod decoder;
pub mod encoder;
pub mod filterbank;
pub mod fixedpoint;
#[cfg(feature = "python")]
mod python;
pub mod synthesis;
pub mod tables;
pub mod wav;

use decoder::{DecoderState, FRAME_SIZE};
use encoder::EncoderState;

/// A1800 audio codec decoder.
///
/// Decodes .a18 bitstream frames into 16-bit PCM audio samples.
/// Each frame produces 320 samples (20ms at 16kHz).
pub struct A1800Decoder {
    state: DecoderState,
}

/// Errors that can occur during decoding.
#[derive(Debug)]
pub enum DecodeError {
    /// Invalid bitrate (must be 4800–32000 in steps of 800).
    InvalidBitrate(u16),
    /// Input buffer too small for one encoded frame.
    InputTooSmall { expected: usize, got: usize },
    /// Output buffer too small for decoded frame.
    OutputTooSmall { expected: usize, got: usize },
}

impl std::fmt::Display for DecodeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            DecodeError::InvalidBitrate(br) => {
                write!(f, "invalid bitrate {}: must be 4800–32000 in steps of 800", br)
            }
            DecodeError::InputTooSmall { expected, got } => {
                write!(f, "input too small: need {} i16 words, got {}", expected, got)
            }
            DecodeError::OutputTooSmall { expected, got } => {
                write!(f, "output too small: need {} samples, got {}", expected, got)
            }
        }
    }
}

impl std::error::Error for DecodeError {}

impl A1800Decoder {
    /// Create a new decoder for the given bitrate.
    ///
    /// Bitrate must be 4800–32000 in steps of 800 (e.g., 8000, 16000, 24000).
    pub fn new(bitrate: u16) -> Result<Self, DecodeError> {
        let state = DecoderState::new(bitrate).map_err(|_| DecodeError::InvalidBitrate(bitrate))?;
        Ok(A1800Decoder { state })
    }

    /// Size of one encoded frame in i16 words.
    pub fn encoded_frame_size(&self) -> usize {
        self.state.encoded_frame_size as usize
    }

    /// Size of one decoded frame in samples (always 320).
    pub fn decoded_frame_size(&self) -> usize {
        FRAME_SIZE
    }

    /// Decode one frame of A1800 audio.
    ///
    /// `input` must contain at least `encoded_frame_size()` i16 words.
    /// `output` must have space for at least 320 samples.
    pub fn decode_frame(
        &mut self,
        input: &[i16],
        output: &mut [i16],
    ) -> Result<(), DecodeError> {
        let enc_size = self.encoded_frame_size();
        if input.len() < enc_size {
            return Err(DecodeError::InputTooSmall {
                expected: enc_size,
                got: input.len(),
            });
        }
        if output.len() < FRAME_SIZE {
            return Err(DecodeError::OutputTooSmall {
                expected: FRAME_SIZE,
                got: output.len(),
            });
        }

        // Decode bitstream into subband samples
        let mut subband = [0i16; FRAME_SIZE];
        let scale_param = self.state.decode_frame_to_subbands(input, &mut subband);

        // Synthesize PCM from subbands
        synthesis::synthesize(
            &subband,
            &mut self.state.synth_memory[..160],
            &mut output[..FRAME_SIZE],
            FRAME_SIZE as i16,
            scale_param,
        );

        Ok(())
    }
}

/// Errors that can occur during encoding.
#[derive(Debug)]
pub enum EncodeError {
    /// Invalid bitrate (must be 4800–32000 in steps of 800).
    InvalidBitrate(u16),
    /// Input buffer too small for one frame.
    InputTooSmall { expected: usize, got: usize },
    /// Output buffer too small for encoded frame.
    OutputTooSmall { expected: usize, got: usize },
}

impl std::fmt::Display for EncodeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            EncodeError::InvalidBitrate(br) => {
                write!(f, "invalid bitrate {}: must be 4800–32000 in steps of 800", br)
            }
            EncodeError::InputTooSmall { expected, got } => {
                write!(f, "input too small: need {} samples, got {}", expected, got)
            }
            EncodeError::OutputTooSmall { expected, got } => {
                write!(f, "output too small: need {} i16 words, got {}", expected, got)
            }
        }
    }
}

impl std::error::Error for EncodeError {}

/// A1800 audio codec encoder.
///
/// Encodes 16-bit PCM audio samples into .a18 bitstream frames.
/// Each frame consumes 320 samples (20ms at 16kHz).
pub struct A1800Encoder {
    state: EncoderState,
}

impl A1800Encoder {
    /// Create a new encoder for the given bitrate.
    ///
    /// Bitrate must be 4800–32000 in steps of 800 (e.g., 8000, 16000, 24000).
    pub fn new(bitrate: u16) -> Result<Self, EncodeError> {
        let state = EncoderState::new(bitrate).map_err(|_| EncodeError::InvalidBitrate(bitrate))?;
        Ok(A1800Encoder { state })
    }

    /// Size of one encoded frame in i16 words.
    pub fn encoded_frame_size(&self) -> usize {
        self.state.encoded_frame_size as usize
    }

    /// Encode one frame of A1800 audio.
    ///
    /// `input` must contain at least 320 PCM samples.
    /// `output` must have space for at least `encoded_frame_size()` i16 words.
    pub fn encode_frame(
        &mut self,
        input: &[i16],
        output: &mut [i16],
    ) -> Result<(), EncodeError> {
        if input.len() < FRAME_SIZE {
            return Err(EncodeError::InputTooSmall {
                expected: FRAME_SIZE,
                got: input.len(),
            });
        }
        let enc_size = self.encoded_frame_size();
        if output.len() < enc_size {
            return Err(EncodeError::OutputTooSmall {
                expected: enc_size,
                got: output.len(),
            });
        }

        // Zero output first
        for w in output[..enc_size].iter_mut() {
            *w = 0;
        }

        self.state.encode_frame_to_bitstream(&input[..FRAME_SIZE], &mut output[..enc_size]);
        Ok(())
    }
}
