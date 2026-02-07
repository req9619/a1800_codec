use std::fs::File;
use std::io::{self, BufWriter, Read, Write};
use std::process;

use a1800_codec::wav::{self, WavWriter};
use a1800_codec::{A1800Decoder, A1800Encoder};

fn main() {
    let args: Vec<String> = std::env::args().collect();

    if args.len() < 2 {
        print_usage();
        process::exit(1);
    }

    match args[1].as_str() {
        "encode" => {
            if args.len() < 4 {
                eprintln!("Usage: a1800_codec encode <input.wav> <output.a18> [--bitrate N]");
                process::exit(1);
            }
            let input_path = &args[2];
            let output_path = &args[3];

            let mut bitrate = 16000u16;
            let mut i = 4;
            while i < args.len() {
                match args[i].as_str() {
                    "--bitrate" => {
                        i += 1;
                        if i >= args.len() {
                            eprintln!("Error: --bitrate requires a value");
                            process::exit(1);
                        }
                        bitrate = args[i].parse().unwrap_or_else(|_| {
                            eprintln!("Error: invalid bitrate '{}'", args[i]);
                            process::exit(1);
                        });
                    }
                    other => {
                        eprintln!("Error: unknown option '{}'", other);
                        process::exit(1);
                    }
                }
                i += 1;
            }

            if let Err(e) = encode_file(input_path, output_path, bitrate) {
                eprintln!("Error: {}", e);
                process::exit(1);
            }
        }
        "decode" => {
            if args.len() < 4 {
                eprintln!("Usage: a1800_codec decode <input.a18> <output.wav> [--sample-rate N]");
                process::exit(1);
            }
            let input_path = &args[2];
            let output_path = &args[3];

            let mut sample_rate = 16000u32;
            let mut i = 4;
            while i < args.len() {
                match args[i].as_str() {
                    "--sample-rate" => {
                        i += 1;
                        if i >= args.len() {
                            eprintln!("Error: --sample-rate requires a value");
                            process::exit(1);
                        }
                        sample_rate = args[i].parse().unwrap_or_else(|_| {
                            eprintln!("Error: invalid sample rate '{}'", args[i]);
                            process::exit(1);
                        });
                    }
                    other => {
                        eprintln!("Error: unknown option '{}'", other);
                        process::exit(1);
                    }
                }
                i += 1;
            }

            if let Err(e) = decode_file(input_path, output_path, sample_rate) {
                eprintln!("Error: {}", e);
                process::exit(1);
            }
        }
        "--help" | "-h" | "help" => {
            print_usage();
        }
        other => {
            eprintln!("Error: unknown command '{}'", other);
            print_usage();
            process::exit(1);
        }
    }
}

fn print_usage() {
    eprintln!("A1800 audio codec");
    eprintln!();
    eprintln!("Usage:");
    eprintln!("  a1800_codec encode <input.wav> <output.a18> [--bitrate N]");
    eprintln!("  a1800_codec decode <input.a18> <output.wav> [--sample-rate N]");
    eprintln!();
    eprintln!("Options:");
    eprintln!("  --bitrate N       Encoding bitrate in bps (default: 16000)");
    eprintln!("  --sample-rate N   Output sample rate in Hz (default: 16000)");
}

fn encode_file(input_path: &str, output_path: &str, bitrate: u16) -> io::Result<()> {
    // Read WAV file
    let mut wav_file = File::open(input_path)?;
    let (samples, sample_rate) = wav::read_wav_samples(&mut wav_file)?;

    eprintln!(
        "WAV: {} samples, {} Hz, {:.1}ms",
        samples.len(),
        sample_rate,
        samples.len() as f64 / sample_rate as f64 * 1000.0
    );

    // Create encoder
    let mut encoder = A1800Encoder::new(bitrate).map_err(|e| {
        io::Error::new(io::ErrorKind::InvalidData, e.to_string())
    })?;

    let enc_frame_words = encoder.encoded_frame_size();
    let enc_frame_bytes = enc_frame_words * 2;
    let num_frames = samples.len() / 320;

    eprintln!(
        "Encoding {} frames at {} bps ({} bytes/frame)...",
        num_frames, bitrate, enc_frame_bytes
    );

    // Write .a18 file
    let mut out_file = File::create(output_path)?;

    // Header: data_length (u32 LE) + bitrate (u16 LE)
    let data_length = (num_frames * enc_frame_bytes) as u32;
    out_file.write_all(&data_length.to_le_bytes())?;
    out_file.write_all(&bitrate.to_le_bytes())?;

    // Encode frames
    let mut frame_words = vec![0i16; enc_frame_words];
    for frame_idx in 0..num_frames {
        let pcm_offset = frame_idx * 320;
        let pcm_frame = &samples[pcm_offset..pcm_offset + 320];

        encoder
            .encode_frame(pcm_frame, &mut frame_words)
            .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e.to_string()))?;

        // Write frame as LE bytes
        for &word in &frame_words {
            out_file.write_all(&word.to_le_bytes())?;
        }
    }

    let total_bytes = 6 + data_length as usize;
    eprintln!(
        "Wrote {} frames ({} bytes) to {}",
        num_frames, total_bytes, output_path
    );

    Ok(())
}

fn decode_file(input_path: &str, output_path: &str, sample_rate: u32) -> io::Result<()> {
    // Read entire input file
    let mut input_data = Vec::new();
    File::open(input_path)?.read_to_end(&mut input_data)?;

    if input_data.len() < 6 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "file too small for A18 header (need at least 6 bytes)",
        ));
    }

    // Parse .a18 header
    let data_length = u32::from_le_bytes(input_data[0..4].try_into().unwrap()) as usize;
    let bitrate = u16::from_le_bytes(input_data[4..6].try_into().unwrap());

    eprintln!(
        "A18: data_length={} bytes, bitrate={} bps",
        data_length, bitrate
    );

    // Create decoder
    let mut decoder = A1800Decoder::new(bitrate).map_err(|e| {
        io::Error::new(io::ErrorKind::InvalidData, e.to_string())
    })?;

    let enc_frame_words = decoder.encoded_frame_size();
    let enc_frame_bytes = enc_frame_words * 2;

    eprintln!(
        "Frame size: {} i16 words ({} bytes), output: {} samples/frame",
        enc_frame_words,
        enc_frame_bytes,
        decoder.decoded_frame_size()
    );

    // Payload starts after 6-byte header
    let payload = &input_data[6..];
    let actual_data_len = payload.len().min(data_length);

    if actual_data_len < enc_frame_bytes {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "no complete frames in file",
        ));
    }

    let num_frames = actual_data_len / enc_frame_bytes;
    eprintln!("Decoding {} frames...", num_frames);

    // Open output WAV
    let out_file = File::create(output_path)?;
    let mut wav = WavWriter::new(BufWriter::new(out_file), sample_rate)?;

    // Convert payload bytes to i16 words (little-endian)
    let mut frame_words = vec![0i16; enc_frame_words];
    let mut pcm_output = [0i16; 320];

    for frame_idx in 0..num_frames {
        let byte_offset = frame_idx * enc_frame_bytes;
        let frame_bytes = &payload[byte_offset..byte_offset + enc_frame_bytes];

        // Convert LE bytes to i16 words
        for (j, word) in frame_words.iter_mut().enumerate() {
            *word = i16::from_le_bytes(frame_bytes[j * 2..j * 2 + 2].try_into().unwrap());
        }

        // Decode frame
        decoder
            .decode_frame(&frame_words, &mut pcm_output)
            .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e.to_string()))?;

        // Write PCM to WAV
        wav.write_samples(&pcm_output)?;
    }

    wav.finish()?;

    let total_samples = num_frames * 320;
    let duration_ms = total_samples as f64 / sample_rate as f64 * 1000.0;
    eprintln!(
        "Wrote {} samples ({:.1}ms) to {}",
        total_samples, duration_ms, output_path
    );

    Ok(())
}
