extern crate tobj;

use nalgebra::Vector3;
use search::IntersectsBVHShape;

use crate::vector3_from_array;
use std::collections::HashMap;


pub struct Material
{
    ambient: Vector3<f32>,
    diffuse: Vector3<f32>,
    pub transmission: Vector3<f32>,
    pub specular: Vector3<f32>,
    pub shininess: f32,
    pub optical_density: f32,
    pub illumination_model: u8,
    diffuse_tex: Option<usize>,
}

pub struct Texture
{
    pub width: f32,
    pub height: f32,
    pub data: Vec<Vector3<f32>>,
}

impl Texture
{
    pub fn new(width: u32, height: u32, data: Vec<Vector3<f32>>) -> Texture
    {
        Texture
        {
            width: width as f32,
            height: height as f32,
            data: data,
        }
    }

    pub fn new_empty() -> Texture
    {
        Texture
        {
            width: 0.0,
            height: 0.0,
            data: Vec::new(),
        }
    }
}

fn load_unknown_param(unknowns: &HashMap<String, String>, param: &String, default: &String) -> Vec<f32> {
    let s= unknowns.get(param).unwrap_or(&default).split_ascii_whitespace();
    s.map(|f| f.parse::<f32>().unwrap()).collect()
}

impl Material
{
    pub fn new(m: &tobj::Material, texs: &std::collections::HashMap<&str, usize>) -> Material
    {
        let tf = load_unknown_param(&m.unknown_param, &"Tf".to_string(), &"1 1 1".to_string());
        assert_eq!(tf.len(), 3, "Tf parameter expected to have 3 channels");
        let tf = Vector3::new(tf[0], tf[1], tf[2]);

        Material
        {
            ambient: vector3_from_array(&m.ambient),
            diffuse: vector3_from_array(&m.diffuse),
            transmission: tf,
            specular: vector3_from_array(&m.specular),
            shininess: m.shininess,
            optical_density: m.optical_density,
            illumination_model: m.illumination_model.unwrap_or(2),
            diffuse_tex: match &m.diffuse_texture[..] {
                "" => None,
                v => Some(*texs.get::<str>(v).unwrap_or(&0)),
            },
        }
    }

    pub fn get_ambient(&self) -> Vector3<f32>
    {
        self.ambient
    }

    pub fn get_diffuse(&self, textures: &Vec<Texture>, shape: &dyn IntersectsBVHShape, u: f32, v: f32) -> Vector3<f32>
    {
        if let Some(tex) = self.diffuse_tex
        {
            let coords = shape.get_tex_coords(u, v);
            let texture = &textures[tex];

            let x = coords.x * texture.width;
            let y = ((texture.height - coords.y * texture.height) as isize) * texture.width as isize;

            let pos = (x as isize) + (y as isize);

            texture.data[pos as usize]
        } else {
            self.diffuse
        }
    }
}
