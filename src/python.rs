use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyByteArray;

use crate::decoder::FRAME_SIZE;
use crate::{A1800Decoder, A1800Encoder};

const FRAME_BYTES: usize = FRAME_SIZE * 2;

/// Stateful A1800 encoder for frame-by-frame encoding.
///
/// Each call to encode_frame() consumes 640 bytes (320 i16 LE samples)
/// and produces encoded frame bytes.
#[pyclass]
struct Encoder {
    inner: A1800Encoder,
}

#[pymethods]
impl Encoder {
    #[new]
    #[pyo3(signature = (bitrate=16000))]
    fn new(bitrate: u16) -> PyResult<Self> {
        let inner =
            A1800Encoder::new(bitrate).map_err(|e| PyValueError::new_err(e.to_string()))?;
        Ok(Encoder { inner })
    }

    /// Size of one encoded frame in bytes.
    #[getter]
    fn encoded_frame_size(&self) -> usize {
        self.inner.encoded_frame_size() * 2
    }

    /// Encode one frame of PCM audio.
    ///
    /// Takes 640 bytes (320 little-endian i16 samples) and returns
    /// the encoded frame as a bytearray.
    fn encode_frame<'py>(
        &mut self,
        py: Python<'py>,
        pcm_data: &[u8],
    ) -> PyResult<Bound<'py, PyByteArray>> {
        if pcm_data.len() < FRAME_BYTES {
            return Err(PyValueError::new_err(format!(
                "need {} bytes (320 i16 LE samples), got {}",
                FRAME_BYTES,
                pcm_data.len()
            )));
        }

        let samples: Vec<i16> = pcm_data[..FRAME_BYTES]
            .chunks_exact(2)
            .map(|c| i16::from_le_bytes([c[0], c[1]]))
            .collect();

        let enc_words = self.inner.encoded_frame_size();
        let mut frame_words = vec![0i16; enc_words];

        self.inner
            .encode_frame(&samples, &mut frame_words)
            .map_err(|e| PyValueError::new_err(e.to_string()))?;

        let mut output = Vec::with_capacity(enc_words * 2);
        for &word in &frame_words {
            output.extend_from_slice(&word.to_le_bytes());
        }

        Ok(PyByteArray::new(py, &output))
    }
}

/// Stateful A1800 decoder for frame-by-frame decoding.
///
/// Each call to decode_frame() consumes encoded frame bytes and
/// produces 640 bytes (320 i16 LE samples).
#[pyclass]
struct Decoder {
    inner: A1800Decoder,
}

#[pymethods]
impl Decoder {
    #[new]
    fn new(bitrate: u16) -> PyResult<Self> {
        let inner =
            A1800Decoder::new(bitrate).map_err(|e| PyValueError::new_err(e.to_string()))?;
        Ok(Decoder { inner })
    }

    /// Size of one encoded frame in bytes.
    #[getter]
    fn encoded_frame_size(&self) -> usize {
        self.inner.encoded_frame_size() * 2
    }

    /// Decode one frame of A1800 audio.
    ///
    /// Takes encoded frame bytes and returns 640 bytes
    /// (320 little-endian i16 PCM samples) as a bytearray.
    fn decode_frame<'py>(
        &mut self,
        py: Python<'py>,
        frame_data: &[u8],
    ) -> PyResult<Bound<'py, PyByteArray>> {
        let enc_words = self.inner.encoded_frame_size();
        let enc_bytes = enc_words * 2;

        if frame_data.len() < enc_bytes {
            return Err(PyValueError::new_err(format!(
                "need {} bytes for frame, got {}",
                enc_bytes,
                frame_data.len()
            )));
        }

        let frame_words: Vec<i16> = frame_data[..enc_bytes]
            .chunks_exact(2)
            .map(|c| i16::from_le_bytes([c[0], c[1]]))
            .collect();

        let mut pcm_output = [0i16; FRAME_SIZE];
        self.inner
            .decode_frame(&frame_words, &mut pcm_output)
            .map_err(|e| PyValueError::new_err(e.to_string()))?;

        let mut output = Vec::with_capacity(FRAME_BYTES);
        for &sample in &pcm_output {
            output.extend_from_slice(&sample.to_le_bytes());
        }

        Ok(PyByteArray::new(py, &output))
    }
}

/// Encode raw PCM audio to .a18 format.
///
/// Takes raw PCM data as bytes (little-endian i16 samples) and a bitrate,
/// returns a bytearray containing the complete .a18 file (header + frames).
/// Samples that don't fill a complete 320-sample frame are discarded.
#[pyfunction]
#[pyo3(signature = (pcm_data, bitrate=16000))]
fn encode<'py>(
    py: Python<'py>,
    pcm_data: &[u8],
    bitrate: u16,
) -> PyResult<Bound<'py, PyByteArray>> {
    if pcm_data.len() % 2 != 0 {
        return Err(PyValueError::new_err(
            "pcm_data length must be even (16-bit samples)",
        ));
    }

    let mut encoder =
        A1800Encoder::new(bitrate).map_err(|e| PyValueError::new_err(e.to_string()))?;

    let num_samples = pcm_data.len() / 2;
    let num_frames = num_samples / FRAME_SIZE;

    let enc_frame_words = encoder.encoded_frame_size();
    let enc_frame_bytes = enc_frame_words * 2;

    let data_length = (num_frames * enc_frame_bytes) as u32;
    let mut output = Vec::with_capacity(6 + data_length as usize);
    output.extend_from_slice(&data_length.to_le_bytes());
    output.extend_from_slice(&bitrate.to_le_bytes());

    let samples: Vec<i16> = pcm_data
        .chunks_exact(2)
        .map(|c| i16::from_le_bytes([c[0], c[1]]))
        .collect();

    let mut frame_words = vec![0i16; enc_frame_words];

    for frame_idx in 0..num_frames {
        let offset = frame_idx * FRAME_SIZE;
        let pcm_frame = &samples[offset..offset + FRAME_SIZE];

        encoder
            .encode_frame(pcm_frame, &mut frame_words)
            .map_err(|e| PyValueError::new_err(e.to_string()))?;

        for &word in &frame_words {
            output.extend_from_slice(&word.to_le_bytes());
        }
    }

    Ok(PyByteArray::new(py, &output))
}

/// Decode .a18 format data to raw PCM audio.
///
/// Takes bytes containing a complete .a18 file (header + frames),
/// returns a bytearray of little-endian i16 PCM samples.
#[pyfunction]
fn decode<'py>(py: Python<'py>, a18_data: &[u8]) -> PyResult<Bound<'py, PyByteArray>> {
    if a18_data.len() < 6 {
        return Err(PyValueError::new_err(
            "data too small for .a18 header (need at least 6 bytes)",
        ));
    }

    let data_length = u32::from_le_bytes(a18_data[0..4].try_into().unwrap()) as usize;
    let bitrate = u16::from_le_bytes(a18_data[4..6].try_into().unwrap());

    let mut decoder =
        A1800Decoder::new(bitrate).map_err(|e| PyValueError::new_err(e.to_string()))?;

    let enc_frame_words = decoder.encoded_frame_size();
    let enc_frame_bytes = enc_frame_words * 2;

    let payload = &a18_data[6..];
    let actual_len = payload.len().min(data_length);

    if actual_len < enc_frame_bytes {
        return Err(PyValueError::new_err("no complete frames in data"));
    }

    let num_frames = actual_len / enc_frame_bytes;
    let mut output = Vec::with_capacity(num_frames * FRAME_BYTES);

    let mut frame_words = vec![0i16; enc_frame_words];
    let mut pcm_output = [0i16; FRAME_SIZE];

    for frame_idx in 0..num_frames {
        let byte_offset = frame_idx * enc_frame_bytes;
        let frame_bytes = &payload[byte_offset..byte_offset + enc_frame_bytes];

        for (j, word) in frame_words.iter_mut().enumerate() {
            *word = i16::from_le_bytes(frame_bytes[j * 2..j * 2 + 2].try_into().unwrap());
        }

        decoder
            .decode_frame(&frame_words, &mut pcm_output)
            .map_err(|e| PyValueError::new_err(e.to_string()))?;

        for &sample in &pcm_output {
            output.extend_from_slice(&sample.to_le_bytes());
        }
    }

    Ok(PyByteArray::new(py, &output))
}

/// GeneralPlus A1800 audio codec.
///
/// Provides encode/decode functions for the A1800 subband audio codec.
/// Audio is 16-bit PCM at 16 kHz, with bitrates from 4800 to 32000 bps.
///
/// Quick usage:
///     import a1800_codec
///     a18_data = a1800_codec.encode(pcm_bytes, bitrate=16000)
///     pcm_bytes = a1800_codec.decode(a18_data)
///
/// Frame-by-frame usage:
///     enc = a1800_codec.Encoder(bitrate=16000)
///     frame = enc.encode_frame(pcm_640_bytes)
///
///     dec = a1800_codec.Decoder(bitrate=16000)
///     pcm = dec.decode_frame(frame)
#[pymodule]
fn a1800_codec(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(encode, m)?)?;
    m.add_function(wrap_pyfunction!(decode, m)?)?;
    m.add_class::<Encoder>()?;
    m.add_class::<Decoder>()?;
    Ok(())
}
