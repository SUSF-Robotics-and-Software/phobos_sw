//! # CSV Archive serializer
//!
//! This module implements `serde::Serializer` for a CSV archive writer.

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

// External imports
use std::fmt::Display;
use serde::{ser, Serialize};
use eyre::WrapErr;
use thiserror::Error;
use color_eyre::Result;
use base64;

// Internal imports

// ---------------------------------------------------------------------------
// DATA STRUCTURES
// ---------------------------------------------------------------------------

/// A serializer for individual records.
pub struct Serializer {
    /// String output for a particular record
    output: String
}

/// A serializer to create a header line from a struct.
pub struct HeaderSerializer {
    /// The header string to output
    output: String,

    /// Counter used to create indices for arrays
    counters: Vec<usize>,

    /// List which will contain the nested names of each struct member
    names: Vec<String>
}

// ---------------------------------------------------------------------------
// ENUMERATIONS
// ---------------------------------------------------------------------------

/// A serialisation error
#[derive(Debug, Error)]
pub enum SerError {
    #[error("{0}")]
    Message(String),

    #[error("Value is not a struct")]
    NotAStruct,
}

// ---------------------------------------------------------------------------
// PUBLIC FUNCTIONS
// ---------------------------------------------------------------------------

pub fn to_string<T>(value: &T) -> Result<String>
where
    T: Serialize
{
    let mut serializer = Serializer {
        output: String::new()
    };

    value.serialize(&mut serializer).wrap_err("Cannot serialize the value")?;
    Ok(serializer.output)
}

pub fn get_header<T>(value: &T) -> Result<String>
where
    T: Serialize
{
    let mut serializer = HeaderSerializer {
        output: String::new(),
        counters: vec![],
        names: vec![]
    };

    value.serialize(&mut serializer)
        .wrap_err("Cannot serialize a header from the value")?;
    Ok(serializer.output)
}

// ---------------------------------------------------------------------------
// IMPLEMENTATIONS
// ---------------------------------------------------------------------------

impl ser::Error for SerError {
    fn custom<T>(msg:T) -> Self
    where T:Display {
        SerError::Message(msg.to_string())
    }
}

impl<'a> ser::Serializer for &'a mut Serializer {
    // Type when ser succeeds
    type Ok = ();

    // Type when ser fails
    type Error = SerError;

    // Associated types for keeping track of additional state while serializing
    // compound data structures like sequences and maps. In this case no
    // additional state is required beyond what is already stored in the
    // Serializer struct.
    type SerializeSeq = Self;
    type SerializeTuple = Self;
    type SerializeTupleStruct = Self;
    type SerializeTupleVariant = Self;
    type SerializeMap = Self;
    type SerializeStruct = Self;
    type SerializeStructVariant = Self;

    fn serialize_bool(self, v: bool) -> Result<Self::Ok, Self::Error> {
        self.output += if v { "TRUE" } else { "FALSE" };
        Ok(())
    }

    fn serialize_i64(self, v: i64) -> Result<Self::Ok, Self::Error> {
        self.output += &format!("{}", v);
        Ok(())
    }

    fn serialize_i32(self, v: i32) -> Result<Self::Ok, Self::Error> {
        self.serialize_i64(i64::from(v))
    }

    fn serialize_i16(self, v: i16) -> Result<Self::Ok, Self::Error> {
        self.serialize_i64(i64::from(v))
    }

    fn serialize_i8(self, v: i8) -> Result<Self::Ok, Self::Error> {
        self.serialize_i64(i64::from(v))
    }

    fn serialize_u64(self, v: u64) -> Result<Self::Ok, Self::Error> {
        self.output += &format!("{}", v);
        Ok(())
    }

    fn serialize_u32(self, v: u32) -> Result<Self::Ok, Self::Error> {
        self.serialize_u64(u64::from(v))
    }

    fn serialize_u16(self, v: u16) -> Result<Self::Ok, Self::Error> {
        self.serialize_u64(u64::from(v))
    }

    fn serialize_u8(self, v: u8) -> Result<Self::Ok, Self::Error> {
        self.serialize_u64(u64::from(v))
    }

    fn serialize_f64(self, v: f64) -> Result<Self::Ok, Self::Error> {
        self.output += &format!("{:.010}", v);
        Ok(())
    }

    fn serialize_f32(self, v: f32) -> Result<Self::Ok, Self::Error> {
        self.serialize_f64(f64::from(v))
    }

    fn serialize_bytes(self, v: &[u8]) -> Result<Self::Ok, Self::Error> {
        self.output += &base64::encode(v);
        Ok(())
    }

    fn serialize_str(self, v: &str) -> Result<Self::Ok, Self::Error> {
        self.output += &format!(r#""{}""#, v);
        Ok(())
    }

    fn serialize_char(self, v: char) -> Result<Self::Ok, Self::Error> {
        self.serialize_str(&format!("{}", v))
    }

    fn serialize_none(self) -> Result<Self::Ok, Self::Error> {
        self.output += "NONE";
        Ok(())
    }

    fn serialize_some<T: ?Sized>(
        self, 
        value: &T
    ) -> Result<Self::Ok, Self::Error>
    where 
        T: Serialize 
    {
        value.serialize(self)    
    }

    fn serialize_unit(self) -> Result<Self::Ok, Self::Error> {
        self.output += "NULL";
        Ok(())
    }

    fn serialize_unit_struct(
        self, _name: &'static str
    ) -> Result<Self::Ok, Self::Error> {
        self.serialize_unit()
    }

    fn serialize_unit_variant(
        self, 
        _name: &'static str, 
        _variant_index: u32, 
        variant: &'static str
    ) -> Result<Self::Ok, Self::Error> {
        self.serialize_str(variant)    
    }

    fn serialize_newtype_struct<T: ?Sized>(
        self, 
        _name: &'static str, 
        value: &T
    ) -> Result<Self::Ok, Self::Error>
    where 
        T: Serialize 
    {
        value.serialize(self)
    }

    fn serialize_newtype_variant<T: ?Sized>(
        self, 
        _name: &'static str, 
        _variant_index: u32, 
        variant: &'static str, 
        value: &T
    ) -> Result<Self::Ok, Self::Error>
    where 
        T: Serialize 
    {
        variant.serialize(&mut *self)?;
        self.output += "(";
        value.serialize(&mut *self)?;
        self.output += ")";
        Ok(())
    }

    fn serialize_seq(
        self, 
        _len: Option<usize>
    ) -> Result<Self::SerializeSeq, Self::Error> {
        Ok(self)
    }

    fn serialize_tuple(
        self, 
        _len: usize
    ) -> Result<Self::SerializeTuple, Self::Error> {
        Ok(self)   
    }

    fn serialize_tuple_variant(
        self, 
        _name: &'static str, 
        _variant_index: u32, 
        _variant: &'static str, 
        _len: usize
    ) -> Result<Self::SerializeTupleVariant, Self::Error> {
        Ok(self)
    }

    fn serialize_tuple_struct(
        self, 
        _name: &'static str, 
        _len: usize
    ) -> Result<Self::SerializeTupleStruct, Self::Error> {
        Ok(self)
    }

    fn serialize_map(
        self,
        _len: Option<usize>
    ) -> Result<Self::SerializeMap, Self::Error> {
        Ok(self)
    }

    fn serialize_struct(
        self, 
        _name: &'static str, 
        _len: usize
    ) -> Result<Self::SerializeStruct, Self::Error> {
        Ok(self)
    }

    fn serialize_struct_variant(
        self, 
        _name: &'static str, 
        _variant_index: u32, 
        _variant: &'static str, 
        _len: usize
    ) -> Result<Self::SerializeStructVariant, Self::Error> {
        Ok(self)   
    }
}

impl<'a> ser::SerializeSeq for &'a mut Serializer {
    type Ok = ();
    type Error = SerError;

    fn serialize_element<T: ?Sized>(
        &mut self, 
        value: &T
    ) -> Result<(), Self::Error>
    where 
        T: Serialize 
    {
        // First element doesn't have comma. If there's already a comma don't
        // add another
        if self.output.len() > 0
            &&
            !self.output.ends_with(", ") 
        {
            self.output += ", ";
        }
        value.serialize(&mut **self)
    }

    fn end(self) -> Result<Self::Ok, Self::Error> {
        Ok(())
    }
}

impl<'a> ser::SerializeTuple for &'a mut Serializer {
    type Ok = ();
    type Error = SerError;

    fn serialize_element<T: ?Sized>(
        &mut self, 
        value: &T
    ) -> Result<(), Self::Error>
    where 
        T: Serialize 
    {
        // First element doesn't have comma. If there's already a comma don't
        // add another
        if self.output.len() > 0
            &&
            !self.output.ends_with(", ") 
        {
            self.output += ", ";
        }
        value.serialize(&mut **self)
    }

    fn end(self) -> Result<Self::Ok, Self::Error> {
        Ok(())
    }
}

impl<'a> ser::SerializeTupleStruct for &'a mut Serializer {
    type Ok = ();
    type Error = SerError;

    fn serialize_field<T: ?Sized>(
        &mut self, 
        value: &T
    ) -> Result<(), Self::Error>
    where 
        T: Serialize 
    {
        // First element doesn't have comma. If there's already a comma don't
        // add another
        if self.output.len() > 0
            &&
            !self.output.ends_with(", ") 
        {
            self.output += ", ";
        }
        value.serialize(&mut **self)
    }

    fn end(self) -> Result<Self::Ok, Self::Error> {
        Ok(())
    }
}

impl<'a> ser::SerializeTupleVariant for &'a mut Serializer {
    type Ok = ();
    type Error = SerError;

    fn serialize_field<T: ?Sized>(
        &mut self, 
        value: &T
    ) -> Result<(), Self::Error>
    where 
        T: Serialize 
    {
        // First element doesn't have comma. If there's already a comma don't
        // add another
        if self.output.len() > 0
            &&
            !self.output.ends_with(", ") 
        {
            self.output += ", ";
        }
        value.serialize(&mut **self)
    }

    fn end(self) -> Result<Self::Ok, Self::Error> {
        Ok(())
    }
}

impl<'a> ser::SerializeMap for &'a mut Serializer {
    type Ok = ();
    type Error = SerError;

    fn serialize_key<T: ?Sized>(
        &mut self, 
        _key: &T
    ) -> Result<(), Self::Error>
    where 
        T: Serialize 
    {
        Ok(())
    }

    fn serialize_value<T: ?Sized>(
        &mut self, 
        value: &T
    ) -> Result<(), Self::Error>
    where 
        T: Serialize 
    {
        // First element doesn't have comma. If there's already a comma don't
        // add another
        if self.output.len() > 0
            &&
            self.output.ends_with(", ") 
        {
            self.output += ", ";
        }
        value.serialize(&mut **self)
    }

    fn end(self) -> Result<Self::Ok, Self::Error> {
        Ok(())
    }
}

impl<'a> ser::SerializeStruct for &'a mut Serializer {
    type Ok = ();
    type Error = SerError;

    fn serialize_field<T: ?Sized>(
        &mut self, 
        _key: &'static str, 
        value: &T
    ) -> Result<(), Self::Error>
    where 
        T: Serialize 
    {
        // First element doesn't have comma. If there's already a comma don't
        // add another
        if self.output.len() > 0
            &&
            !self.output.ends_with(", ") 
        {
            self.output += ", ";
        }
        value.serialize(&mut **self)
    }

    fn end(self) -> Result<Self::Ok, Self::Error> {
        Ok(())
    }
}

impl<'a> ser::SerializeStructVariant for &'a mut Serializer {
    type Ok = ();
    type Error = SerError;

    fn serialize_field<T: ?Sized>(
        &mut self, 
        _key: &'static str, 
        value: &T
    ) -> Result<(), Self::Error>
    where 
        T: Serialize 
    {
        // First element doesn't have comma. If there's already a comma don't
        // add another
        if self.output.len() > 0
            &&
            !self.output.ends_with(", ") 
        {
            self.output += ", ";
        }
        value.serialize(&mut **self)
    }

    fn end(self) -> Result<Self::Ok, Self::Error> {
        Ok(())
    }
}

impl<'a> ser::Serializer for &'a mut HeaderSerializer {

    type Ok = ();
    type Error = SerError;

    type SerializeSeq = Self;
    type SerializeTuple = Self;
    type SerializeTupleStruct = Self;
    type SerializeTupleVariant = Self;
    type SerializeMap = Self;
    type SerializeStruct = Self;
    type SerializeStructVariant = Self;

    fn serialize_bool(self, _v: bool) -> Result<Self::Ok, Self::Error> {
        Err(SerError::NotAStruct)
    }

    fn serialize_bytes(self, _v: &[u8]) -> Result<Self::Ok, Self::Error> {
        Err(SerError::NotAStruct)
    }

    fn serialize_char(self, _v: char) -> Result<Self::Ok, Self::Error> {
        Err(SerError::NotAStruct)
    }

    fn serialize_f64(self, _v: f64) -> Result<Self::Ok, Self::Error> {
        Err(SerError::NotAStruct)
    }

    fn serialize_f32(self, _v: f32) -> Result<Self::Ok, Self::Error> {
        Err(SerError::NotAStruct)
    }

    fn serialize_i64(self, _v: i64) -> Result<Self::Ok, Self::Error> {
        Err(SerError::NotAStruct)
    }

    fn serialize_i32(self, _v: i32) -> Result<Self::Ok, Self::Error> {
        Err(SerError::NotAStruct)
    }

    fn serialize_i16(self, _v: i16) -> Result<Self::Ok, Self::Error> {
        Err(SerError::NotAStruct)
    }

    fn serialize_i8(self, _v: i8) -> Result<Self::Ok, Self::Error> {
        Err(SerError::NotAStruct)
    }

    fn serialize_map(self, _len: Option<usize>) -> Result<Self::SerializeMap, Self::Error> {
        Err(SerError::NotAStruct)
    }

    fn serialize_newtype_struct<T: ?Sized>(self, _name: &'static str, _value: &T) -> Result<Self::Ok, Self::Error>
    where T: Serialize {
        Err(SerError::NotAStruct)
    }

    fn serialize_newtype_variant<T: ?Sized>(self, _name: &'static str, _variant_index: u32, _variant: &'static str, _value: &T) -> Result<Self::Ok, Self::Error>
    where T: Serialize {
        Err(SerError::NotAStruct)
    }

    fn serialize_none(self) -> Result<Self::Ok, Self::Error> {
        Err(SerError::NotAStruct)
    }

    fn serialize_seq(self, _len: Option<usize>) -> Result<Self::SerializeSeq, Self::Error> {
        self.counters.push(0);
        Ok(self)
    }

    fn serialize_some<T: ?Sized>(self, _value: &T) -> Result<Self::Ok, Self::Error>
    where T: Serialize {
        Err(SerError::NotAStruct)
    }

    fn serialize_str(self, _v: &str) -> Result<Self::Ok, Self::Error> {
        Err(SerError::NotAStruct)
    }

    fn serialize_struct(self, _name: &'static str, _len: usize) -> Result<Self::SerializeStruct, Self::Error> {
        Ok(self)
    }

    fn serialize_struct_variant(self, _name: &'static str, _variant_index: u32, _variant: &'static str, _len: usize) -> Result<Self::SerializeStructVariant, Self::Error> {
        Ok(self)
    }

    fn serialize_tuple(self, _len: usize) -> Result<Self::SerializeTuple, Self::Error> {
        self.counters.push(0);
        Ok(self)
    }

    fn serialize_tuple_struct(self, _name: &'static str, _len: usize) -> Result<Self::SerializeTupleStruct, Self::Error> {
        self.counters.push(0);
        Ok(self)
    }

    fn serialize_tuple_variant(self, _name: &'static str, _variant_index: u32, _variant: &'static str, _len: usize) -> Result<Self::SerializeTupleVariant, Self::Error> {
        self.counters.push(0);
        Ok(self)
    }

    fn serialize_u16(self, _v: u16) -> Result<Self::Ok, Self::Error> {
        Err(SerError::NotAStruct)
    }

    fn serialize_u32(self, _v: u32) -> Result<Self::Ok, Self::Error> {
        Err(SerError::NotAStruct)
    }

    fn serialize_u64(self, _v: u64) -> Result<Self::Ok, Self::Error> {
        Err(SerError::NotAStruct)
    }

    fn serialize_u8(self, _v: u8) -> Result<Self::Ok, Self::Error> {
        Err(SerError::NotAStruct)
    }

    fn serialize_unit(self) -> Result<Self::Ok, Self::Error> {
        Err(SerError::NotAStruct)
    }

    fn serialize_unit_struct(self, _name: &'static str) -> Result<Self::Ok, Self::Error> {
        Err(SerError::NotAStruct)
    }

    fn serialize_unit_variant(self, _name: &'static str, _variant_index: u32, _variant: &'static str) -> Result<Self::Ok, Self::Error> {
        Err(SerError::NotAStruct)
    }
}

impl<'a> ser::SerializeStruct for &'a mut HeaderSerializer {
    type Ok = ();
    type Error = SerError;

    fn serialize_field<T: ?Sized>(
        &mut self, 
        key: &'static str, 
        value: &T
    ) -> Result<(), Self::Error>
    where 
        T: Serialize 
    {
        if self.output.len() > 0
            &&
            !self.output.ends_with(", ")
        {
            self.output += ", ";
        }
        self.names.push(String::from(key));
        match value.serialize(&mut **self) {
            Ok(_) => (),
            Err(SerError::NotAStruct) => self.output += &self.names.join("."),
            Err(e) => return Err(e)
        };
        self.names.pop();

        Ok(())
    }

    fn end(self) -> Result<Self::Ok, Self::Error> {
        Ok(())
    }
}

impl<'a> ser::SerializeSeq for &'a mut HeaderSerializer {
    type Ok = ();
    type Error = SerError;

    fn serialize_element<T: ?Sized>(
        &mut self, 
        _value: &T
    ) -> Result<(), Self::Error>
    where 
        T: Serialize 
    {
        if self.output.len() > 0
            &&
            !self.output.ends_with(", ")
        {
            self.output += ", ";
        }

        self.output += &format!(
            "{}[{}]",
            self.names.join("."), 
            self.counters.last().unwrap());

        match self.counters.last_mut() {
            Some(c) => *c += 1,
            None => return Err(SerError::Message(String::from("No counter")))
        }

        Ok(())
    }

    fn end(self) -> Result<Self::Ok, Self::Error> {
        Ok(())
    }
}

impl<'a> ser::SerializeMap for &'a mut HeaderSerializer {
    type Ok = ();
    type Error = SerError;

    fn serialize_key<T: ?Sized>(&mut self, _key: &T) -> Result<(), Self::Error>
    where T: Serialize {
        Ok(())
    }

    fn serialize_value<T: ?Sized>(&mut self, _value: &T) -> Result<(), Self::Error>
    where T: Serialize {
        Ok(())
    }

    fn serialize_entry<K: ?Sized, V: ?Sized>(
        &mut self, 
        key: &K, 
        _value: &V
    ) -> Result<(), Self::Error>
    where 
        K: Serialize,
        V: Serialize 
    {
        if self.output.len() > 0
            &&
            !self.output.ends_with(", ")
        {
            self.output += ", ";
        }
        
        self.output += &self.names.join(".");

        key.serialize(&mut **self)?;

        Ok(())
    }

    fn end(self) -> Result<Self::Ok, Self::Error> {
        Ok(())
    }
}

impl<'a> ser::SerializeStructVariant for &'a mut HeaderSerializer {
    type Ok = ();
    type Error = SerError;
    
    fn serialize_field<T: ?Sized>(
        &mut self, 
        key: &'static str, 
        _value: &T
    ) -> Result<(), Self::Error>
    where 
        T: Serialize 
    {
        if self.output.len() > 0
            &&
            !self.output.ends_with(", ")
        {
            self.output += ", ";
        }
        self.names.push(String::from(key));

        self.output += &self.names.join(".");
        Ok(())
    }

    fn end(self) -> Result<Self::Ok, Self::Error> {
        Ok(())
    }
}

impl<'a> ser::SerializeTuple for &'a mut HeaderSerializer {
    type Ok = ();
    type Error = SerError;

    fn serialize_element<T: ?Sized>(
        &mut self, 
        _value: &T
    ) -> Result<(), Self::Error>
    where 
        T: Serialize 
    {
        if self.output.len() > 0
            &&
            !self.output.ends_with(", ")
        {
            self.output += ", ";
        }

        self.output += &format!(
            "{}[{}]",
            self.names.join("."), 
            self.counters.last().unwrap());

        match self.counters.last_mut() {
            Some(c) => *c += 1,
            None => return Err(SerError::Message(String::from("No counter")))
        }

        Ok(())
    }

    fn end(self) -> Result<Self::Ok, Self::Error> {
        Ok(())
    }
}

impl<'a> ser::SerializeTupleStruct for &'a mut HeaderSerializer {
    type Ok = ();
    type Error = SerError;

    fn serialize_field<T: ?Sized>(
        &mut self, 
        _value: &T
    ) -> Result<(), Self::Error>
    where 
        T: Serialize 
    {
        Ok(())
    }

    fn end(self) -> Result<Self::Ok, Self::Error> {
        Ok(())
    }
}

impl<'a> ser::SerializeTupleVariant for &'a mut HeaderSerializer {
    type Ok = ();
    type Error = SerError;

    fn serialize_field<T: ?Sized>(
        &mut self,
        _value: &T
    ) -> Result<(), Self::Error>
    where 
        T: Serialize 
    {
        Ok(())
    }

    fn end(self) -> Result<Self::Ok, Self::Error> {
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// TESTS
// ---------------------------------------------------------------------------

#[test]
fn test_struct() {
    #[derive(Serialize)]
    struct Test {
        a: i32,
        b: Vec<f64>,
        c: String,
        d: Option<u8>
    }

    let test = Test {
        a: 12,
        b: vec![0.0, 1.0, 2.0],
        c: String::from("Hello world!"),
        d: None
    };

    let expected = r#"12, 0.0000000000, 1.0000000000, 2.0000000000, "Hello world!", NONE"#;
    assert_eq!(to_string(&test).unwrap(), expected);
}

#[test]
fn test_header() {

    #[derive(Serialize)]
    struct Nested {
        in_nested: i32
    }

    #[derive(Serialize)]
    struct Test {
        some_val: i32,
        nested: Vec<Nested>,
        an_array: [i32; 3],
        a_vec: Vec<i32>
    }

    let test = Test {
        some_val: 0,
        nested: vec![Nested { in_nested: 12 }, Nested{ in_nested: 13 }],
        an_array: [0, 1, 2],
        a_vec: vec![1, 2, 3, 4, 5]
    };

    let expected = r#"some_val, nested.in_nested, an_array[0], an_array[1], an_array[2], a_vec[0], a_vec[1], a_vec[2], a_vec[3], a_vec[4]"#;
    assert_eq!(get_header(&test).unwrap(), expected);
}