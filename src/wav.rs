/// Mono 16-bit PCM WAV file reader and writer.

use std::io::{self, Read, Write};

/// Write a complete WAV file with mono 16-bit PCM samples.
pub fn write_wav<W: Write>(
    writer: &mut W,
    samples: &[i16],
    sample_rate: u32,
) -> io::Result<()> {
    let num_channels: u16 = 1;
    let bits_per_sample: u16 = 16;
    let byte_rate = sample_rate * (num_channels as u32) * (bits_per_sample as u32 / 8);
    let block_align = num_channels * (bits_per_sample / 8);
    let data_size = (samples.len() * 2) as u32;
    let riff_size = 36 + data_size;

    // RIFF header
    writer.write_all(b"RIFF")?;
    writer.write_all(&riff_size.to_le_bytes())?;
    writer.write_all(b"WAVE")?;

    // fmt sub-chunk
    writer.write_all(b"fmt ")?;
    writer.write_all(&16u32.to_le_bytes())?; // sub-chunk size
    writer.write_all(&1u16.to_le_bytes())?; // audio format (PCM)
    writer.write_all(&num_channels.to_le_bytes())?;
    writer.write_all(&sample_rate.to_le_bytes())?;
    writer.write_all(&byte_rate.to_le_bytes())?;
    writer.write_all(&block_align.to_le_bytes())?;
    writer.write_all(&bits_per_sample.to_le_bytes())?;

    // data sub-chunk
    writer.write_all(b"data")?;
    writer.write_all(&data_size.to_le_bytes())?;
    for &sample in samples {
        writer.write_all(&sample.to_le_bytes())?;
    }

    Ok(())
}

/// Streaming WAV writer that writes header first, then samples incrementally.
pub struct WavWriter<W: Write> {
    inner: W,
    sample_count: u32,
    sample_rate: u32,
}

impl<W: Write + io::Seek> WavWriter<W> {
    /// Create a new WAV writer. Writes the header immediately (with placeholder sizes).
    pub fn new(mut writer: W, sample_rate: u32) -> io::Result<Self> {
        // Write placeholder header (sizes will be patched on finish)
        let header = [0u8; 44];
        writer.write_all(&header)?;
        Ok(WavWriter {
            inner: writer,
            sample_count: 0,
            sample_rate,
        })
    }

    /// Write a block of samples.
    pub fn write_samples(&mut self, samples: &[i16]) -> io::Result<()> {
        for &s in samples {
            self.inner.write_all(&s.to_le_bytes())?;
        }
        self.sample_count += samples.len() as u32;
        Ok(())
    }

    /// Finalize the WAV file by patching the header with correct sizes.
    pub fn finish(mut self) -> io::Result<W> {
        use std::io::SeekFrom;

        let data_size = self.sample_count * 2;
        let riff_size = 36 + data_size;
        let byte_rate = self.sample_rate * 2; // mono 16-bit
        let sample_rate = self.sample_rate;

        self.inner.seek(SeekFrom::Start(0))?;

        // RIFF header
        self.inner.write_all(b"RIFF")?;
        self.inner.write_all(&riff_size.to_le_bytes())?;
        self.inner.write_all(b"WAVE")?;

        // fmt sub-chunk
        self.inner.write_all(b"fmt ")?;
        self.inner.write_all(&16u32.to_le_bytes())?;
        self.inner.write_all(&1u16.to_le_bytes())?; // PCM
        self.inner.write_all(&1u16.to_le_bytes())?; // mono
        self.inner.write_all(&sample_rate.to_le_bytes())?;
        self.inner.write_all(&byte_rate.to_le_bytes())?;
        self.inner.write_all(&2u16.to_le_bytes())?; // block align
        self.inner.write_all(&16u16.to_le_bytes())?; // bits per sample

        // data sub-chunk
        self.inner.write_all(b"data")?;
        self.inner.write_all(&data_size.to_le_bytes())?;

        Ok(self.inner)
    }
}

/// Read a WAV file and return its samples and sample rate.
///
/// Only supports mono or stereo 16-bit PCM WAV files.
/// Stereo files are downmixed to mono by averaging channels.
pub fn read_wav_samples<R: Read>(reader: &mut R) -> io::Result<(Vec<i16>, u32)> {
    let mut data = Vec::new();
    reader.read_to_end(&mut data)?;

    if data.len() < 44 {
        return Err(io::Error::new(io::ErrorKind::InvalidData, "file too small for WAV header"));
    }

    // Validate RIFF header
    if &data[0..4] != b"RIFF" || &data[8..12] != b"WAVE" {
        return Err(io::Error::new(io::ErrorKind::InvalidData, "not a valid WAV file"));
    }

    // Find fmt chunk
    let mut pos = 12;
    let mut sample_rate = 0u32;
    let mut num_channels = 0u16;
    let mut bits_per_sample = 0u16;
    let mut audio_format = 0u16;
    let mut found_fmt = false;

    while pos + 8 <= data.len() {
        let chunk_id = &data[pos..pos + 4];
        let chunk_size = u32::from_le_bytes(data[pos + 4..pos + 8].try_into().unwrap()) as usize;

        if chunk_id == b"fmt " {
            if chunk_size < 16 || pos + 8 + chunk_size > data.len() {
                return Err(io::Error::new(io::ErrorKind::InvalidData, "invalid fmt chunk"));
            }
            let fmt_data = &data[pos + 8..];
            audio_format = u16::from_le_bytes(fmt_data[0..2].try_into().unwrap());
            num_channels = u16::from_le_bytes(fmt_data[2..4].try_into().unwrap());
            sample_rate = u32::from_le_bytes(fmt_data[4..8].try_into().unwrap());
            bits_per_sample = u16::from_le_bytes(fmt_data[14..16].try_into().unwrap());
            found_fmt = true;
        }

        if chunk_id == b"data" {
            if !found_fmt {
                return Err(io::Error::new(io::ErrorKind::InvalidData, "data chunk before fmt chunk"));
            }
            if audio_format != 1 {
                return Err(io::Error::new(io::ErrorKind::InvalidData, "only PCM format (1) is supported"));
            }
            if bits_per_sample != 16 {
                return Err(io::Error::new(io::ErrorKind::InvalidData, "only 16-bit samples are supported"));
            }

            let data_start = pos + 8;
            let data_end = (data_start + chunk_size).min(data.len());
            let raw = &data[data_start..data_end];

            if num_channels == 1 {
                // Mono: direct conversion
                let num_samples = raw.len() / 2;
                let mut samples = Vec::with_capacity(num_samples);
                for i in 0..num_samples {
                    samples.push(i16::from_le_bytes(raw[i * 2..i * 2 + 2].try_into().unwrap()));
                }
                return Ok((samples, sample_rate));
            } else if num_channels == 2 {
                // Stereo: downmix to mono
                let bytes_per_frame = 4; // 2 channels * 2 bytes
                let num_frames = raw.len() / bytes_per_frame;
                let mut samples = Vec::with_capacity(num_frames);
                for i in 0..num_frames {
                    let left = i16::from_le_bytes(raw[i * 4..i * 4 + 2].try_into().unwrap()) as i32;
                    let right = i16::from_le_bytes(raw[i * 4 + 2..i * 4 + 4].try_into().unwrap()) as i32;
                    samples.push(((left + right) / 2) as i16);
                }
                return Ok((samples, sample_rate));
            } else {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    format!("unsupported channel count: {}", num_channels),
                ));
            }
        }

        pos += 8 + chunk_size;
        // Pad to word boundary
        if chunk_size % 2 != 0 {
            pos += 1;
        }
    }

    Err(io::Error::new(io::ErrorKind::InvalidData, "no data chunk found in WAV file"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_write_wav_header() {
        let samples = [0i16; 320];
        let mut buf = Vec::new();
        write_wav(&mut buf, &samples, 16000).unwrap();

        // Check RIFF header
        assert_eq!(&buf[0..4], b"RIFF");
        let riff_size = u32::from_le_bytes(buf[4..8].try_into().unwrap());
        assert_eq!(riff_size, 36 + 640); // 320 samples * 2 bytes
        assert_eq!(&buf[8..12], b"WAVE");

        // Check fmt
        assert_eq!(&buf[12..16], b"fmt ");
        let fmt_size = u32::from_le_bytes(buf[16..20].try_into().unwrap());
        assert_eq!(fmt_size, 16);
        let audio_fmt = u16::from_le_bytes(buf[20..22].try_into().unwrap());
        assert_eq!(audio_fmt, 1); // PCM

        // Check data
        assert_eq!(&buf[36..40], b"data");
        let data_size = u32::from_le_bytes(buf[40..44].try_into().unwrap());
        assert_eq!(data_size, 640);

        // Total size
        assert_eq!(buf.len(), 44 + 640);
    }

    #[test]
    fn test_read_wav_roundtrip() {
        // Write a WAV, then read it back
        let samples = [100i16, -200, 300, -400, 500];
        let mut buf = Vec::new();
        write_wav(&mut buf, &samples, 16000).unwrap();

        let mut cursor = std::io::Cursor::new(&buf);
        let (read_samples, sr) = read_wav_samples(&mut cursor).unwrap();
        assert_eq!(sr, 16000);
        assert_eq!(read_samples, samples);
    }
}
