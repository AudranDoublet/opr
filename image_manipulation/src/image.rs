extern crate image;

use std::path::Path;

use nalgebra::Vector3;
use rayon::prelude::*;

use crate::*;

#[derive(Copy, Clone, Debug)]
pub enum ImageMode
{
    RGB,
    HSV,
    Grayscale,
}

pub struct Image
{
    mode: ImageMode,
    pub pixels: Vec<u16>,
    width: usize,
    height: usize,
}

#[inline]
fn pix_to_gray(r: u16, g: u16, b: u16) -> u16
{
    ((r as u16 + g as u16 + b as u16) / 3) as u16
}

#[inline]
pub fn pix_clamp(pix: u16, min: u16, max: u16) -> u16
{
    match pix {
        pix if pix < min => min,
        pix if pix > max => max,
        pix => pix,
    }
}

#[inline]
pub fn ipix_clamp(pix: i16, min: i16, max: i16) -> i16
{
    match pix {
        pix if pix < min => min,
        pix if pix > max => max,
        pix => pix,
    }
}

#[inline]
pub fn fpix_clamp(pix: f32) -> u16
{
    ((u8::max_value() as f32 * pix.max(0.0)) as u16).min(u8::max_value() as u16)
}

impl Image
{
    pub fn load(path: &Path) -> Result<Image, Box<dyn std::error::Error>>
    {
        let image = image::open(path)?.to_rgb();

        let mut result = vec![];

        for pix in image.pixels()
        {
            let data = pix;

            result.push(data[0] as u16);
            result.push(data[1] as u16);
            result.push(data[2] as u16);
        }

        Ok(Image {
            mode: ImageMode::RGB,
            pixels: result,
            width: image.width() as usize,
            height: image.height() as usize,
        })
    }

    pub fn from_vectors(pixels: &Vec<Vector3<f32>>, width: usize, height: usize) -> Image
    {
        assert_eq!(width * height, pixels.len());

        let mut result = vec![0; pixels.len() * 3];

        for i in 0..pixels.len()
        {
            let data = pixels[i];

            result[i * 3 + 0] = fpix_clamp(data.x);
            result[i * 3 + 1] = fpix_clamp(data.y);
            result[i * 3 + 2] = fpix_clamp(data.z);
        }

        Image {
            mode: ImageMode::RGB,
            pixels: result,
            width: width,
            height: height,
        }
    }

    pub fn save(&self, path: &Path)
    {
        let mut buf = Vec::new();

        for v in &self.pixels {
            buf.push(*v as u8);
        }

        match image::save_buffer(
            path, &buf, self.width as u32, self.height as u32,
            match self.mode {
                ImageMode::Grayscale => image::ColorType::L8,
                ImageMode::RGB => image::ColorType::Rgb8,
                _ => panic!("can only save RGB & Grayscale images"),
            },
        ) {
            Ok(_) => (),
            Err(x) => panic!("can't save image: {:?}", x),
        }
    }

    pub fn copy(&self) -> Image {
        let mut pixels = vec![0; self.pixels.len()];
        pixels.clone_from_slice(&self.pixels);

        Image {
            pixels: pixels,
            width: self.width,
            height: self.height,
            mode: self.mode,
        }
    }

    pub fn to_grayscale(&self) -> Image
    {
        let mut result = Vec::new();

        self.foreach_rgb(&mut |r, g, b| result.push(pix_to_gray(r, g, b) / 3));

        Image {
            mode: ImageMode::Grayscale,
            pixels: result,
            width: self.width,
            height: self.height,
        }
    }

    pub fn to_hsv(&self) -> Image
    {
        let mut result = Vec::new();

        self.foreach_rgb(&mut |r, g, b| {
            let vr = r as f32 / 255.;
            let vg = g as f32 / 255.;
            let vb = b as f32 / 255.;

            let vmin = vr.min(vg).min(vb);
            let (t, vmax);

            if vr > vg && vr > vb {
                vmax = vr;
                t = (60. * (vg - vb) / (vmax - vmin) + 360.) % 360.;
            } else if vg > vr && vg > vb {
                vmax = vb;
                t = (60. * (vb - vr) / (vmax - vmin) + 120.) % 360.;
            } else if vb > vg && vb > vr {
                vmax = vb;
                t = (60. * (vr - vg) / (vmax - vmin) + 240.) % 360.;
            } else {
                t = 0.0;
                vmax = -1.0;
            }

            result.push(t as u16);
            result.push((match vmax {
                v if v <= 0.00001 => 0.,
                _ => 1. - vmin / vmax
            } * 255.) as u16);
            result.push((vmax * 255.) as u16);
        });

        Image {
            mode: ImageMode::HSV,
            pixels: result,
            width: self.width,
            height: self.height,
        }
    }

    pub fn from_hsv(&self) -> Image
    {
        let mut result = Vec::new();

        self.foreach_rgb_f(&mut |t, s, v| {
            let t = (t * 255.) as u16;

            let ti = (t / 60) % 6;
            let f = (t as f32 / 60.) - ti as f32;

            let l = v * (1. - s);
            let m = v * (1. - f * s);
            let n = v * (1. - (1. - f) * s);

            let (r, g, b) = match ti {
                0 => (v, n, l),
                1 => (m, v, l),
                2 => (l, v, n),
                3 => (l, m, v),
                4 => (n, l, v),
                _ => (v, l, m),
            };

            result.push((r * 255.) as u16);
            result.push((g * 255.) as u16);
            result.push((b * 255.) as u16);
        });

        Image {
            mode: ImageMode::RGB,
            pixels: result,
            width: self.width,
            height: self.height,
        }
    }

    pub fn channels(&self) -> usize
    {
        match self.mode
        {
            ImageMode::Grayscale => 1,
            _ => 3,
        }
    }

    pub fn foreach_channel(&self, channel: usize, f: &mut dyn FnMut(u16))
    {
        let step = self.channels();

        for i in (channel..self.pixels.len()).step_by(step)
        {
            f(self.pixels[i]);
        }
    }

    pub fn basic_apply(&mut self, f: &dyn Fn(usize) -> u16)
    {
        for i in 0..self.pixels.len()
        {
            self.pixels[i] = f(i);
        }
    }

    pub fn apply_channel(&mut self, channel: usize, f: &mut dyn FnMut(u16) -> u16)
    {
        let step = self.channels();

        for i in (channel..self.pixels.len()).step_by(step)
        {
            self.pixels[i] = f(self.pixels[i]);
        }
    }

    pub fn foreach_rgb(&self, f: &mut dyn FnMut(u16, u16, u16))
    {
        let step = self.channels();

        for i in (0..self.pixels.len()).step_by(step)
        {
            let v = &self.pixels[i..i + step];
            f(v[0], v[1], v[2]);
        }
    }

    pub fn foreach_rgb_f(&self, f: &mut dyn FnMut(f32, f32, f32))
    {
        let step = self.channels();

        for i in (0..self.pixels.len()).step_by(step)
        {
            let v = &self.pixels[i..i + step];
            f(v[0] as f32 / 255., v[1] as f32 / 255., v[2] as f32 / 255.);
        }
    }

    pub fn histogram_channel(&self, channel: usize) -> Histogram
    {
        let mut histogram = Histogram::new();

        self.foreach_channel(channel, &mut |v| histogram.add(v));

        histogram
    }

    pub fn white_balance(&mut self, channel: usize, threshold_perc: f32)
    {
        let histogram = self.histogram_channel(channel);
        let threshold = (histogram.get_total() as f32 * threshold_perc) as usize;

        let (min, max) = histogram.threshold(threshold);

        self.apply_channel(channel,
                           &mut |v| (pix_clamp(v, min, max) - min) * 255 / (max - min));
    }

    pub fn hist_balance(&mut self, channel: usize)
    {
        let histogram = self.histogram_channel(channel);
        let cum = histogram.cumulative();

        self.apply_channel(channel, &mut |v| ((255 * cum.at(v)) / histogram.get_total()) as u16);
    }

    pub fn channel_at(&self, x: usize, y: usize, channel: usize) -> u16 {
        self.pixels[(x + y * self.width) * self.channels() + channel]
    }

    pub fn apply_gamma_correction(&mut self, gamma: f32) {
        self.apply_gamma(1. / gamma)
    }

    pub fn apply_gamma(&mut self, gamma: f32) {
        for channel in 0..self.channels() {
            self.apply_channel(channel, &mut |v| (255. * (v as f32 / 255.).powf(gamma)) as u16);
        }
    }

    pub fn convolute(&self, channel: usize, matrix: &ConvMatrix) -> Image
    {
        let mut pixels = vec![0; self.pixels.len()];
        pixels.clone_from_slice(&self.pixels);

        let mut slice = vec![0.; 9];

        for x in 1..self.width - 1 {
            for y in 1..self.height - 1 {
                slice[0] = self.channel_at(x - 1, y - 1, channel) as f32;
                slice[1] = self.channel_at(x + 0, y - 1, channel) as f32;
                slice[2] = self.channel_at(x + 1, y - 1, channel) as f32;
                slice[3] = self.channel_at(x - 1, y + 0, channel) as f32;
                slice[4] = self.channel_at(x + 0, y + 0, channel) as f32;
                slice[5] = self.channel_at(x + 1, y + 0, channel) as f32;
                slice[6] = self.channel_at(x - 1, y + 1, channel) as f32;
                slice[7] = self.channel_at(x + 0, y + 1, channel) as f32;
                slice[8] = self.channel_at(x + 1, y + 1, channel) as f32;
                pixels[self.channels() * (y * self.width + x) + channel] = fpix_clamp(matrix.apply(&slice));
            }
        }

        Image {
            pixels: pixels,
            width: self.width,
            height: self.height,
            mode: self.mode,
        }
    }

    pub fn convolute_all_channels(&self, matrix: &ConvMatrix) -> Image
    {
        let mut pixels = vec![0; self.pixels.len()];
        pixels.clone_from_slice(&self.pixels);

        let channels = self.channels();

        pixels.par_iter_mut()
            .enumerate()
            .for_each(|(i, p)| {
                let channel = i % channels;
                let x = i / channels;

                let y = x / self.width;
                let x = x % self.width;

                if x == 0 || x == self.width - 1 || y == 0 || y == self.height - 1 {
                    return;
                }

                let slice = vec![
                    self.channel_at(x - 1, y - 1, channel) as f32,
                    self.channel_at(x + 0, y - 1, channel) as f32,
                    self.channel_at(x + 1, y - 1, channel) as f32,
                    self.channel_at(x - 1, y + 0, channel) as f32,
                    self.channel_at(x + 0, y + 0, channel) as f32,
                    self.channel_at(x + 1, y + 0, channel) as f32,
                    self.channel_at(x - 1, y + 1, channel) as f32,
                    self.channel_at(x + 0, y + 1, channel) as f32,
                    self.channel_at(x + 1, y + 1, channel) as f32,
                ];

                *p = fpix_clamp(matrix.apply(&slice));
            });

        Image {
            pixels: pixels,
            width: self.width,
            height: self.height,
            mode: self.mode,
        }
    }

    pub fn sobel(&self) -> Image {
        let gx = ConvMatrix::new_3x3(vec![-1., 0., 1., -2., 0., 2., -1., 0., 1.]);
        let gy = ConvMatrix::new_3x3(vec![-1., -2., -1., 0., 0., 0., 1., 2., 1.]);

        let c1 = self.convolute_all_channels(&gx).pixels;
        let c2 = self.convolute_all_channels(&gy).pixels;

        let mut pixels = vec![0; self.pixels.len()];

        pixels.par_iter_mut().enumerate().for_each(|(i, p)| {
            *p = ((c1[i].pow(2) + c2[i].pow(2)) as f32).sqrt() as u16;
        });

        Image {
            pixels: pixels,
            width: self.width,
            height: self.height,
            mode: self.mode,
        }
    }
}
