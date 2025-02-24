#[derive(Debug, Clone, Default)]
pub struct PgmImage {
    pub width: usize,
    pub height: usize,
    pub max_value: u16,
    pub data: Vec<u8>,
}

impl PgmImage {
    pub fn iter(&self) -> impl Iterator<Item = (usize, usize, u8)> + '_ {
        self.data.iter().enumerate().map(move |(i, &value)| {
            let x = i % self.width;
            let y = i / self.width;
            (x, y, value)
        })
    }
}

pub struct Parser {
    bytes: Vec<u8>,
    cursor: usize,
}

impl Parser {
    fn consume(&mut self) -> u8 {
        let byte = self.bytes[self.cursor];
        self.cursor += 1;
        byte
    }

    fn consume_str(&mut self, len: usize) -> String {
        let start = self.cursor;
        self.cursor += len;
        String::from_utf8(self.bytes[start..self.cursor].to_vec()).unwrap()
    }

    fn consume_str_u16(&mut self) -> u16 {
        let mut value = 0;
        while self.cursor < self.bytes.len() && self.bytes[self.cursor].is_ascii_digit() {
            value = value * 10 + (self.bytes[self.cursor] - b'0') as u16;
            self.cursor += 1;
        }
        value
    }

    fn consume_whitespace(&mut self) {
        while self.cursor < self.bytes.len() && self.bytes[self.cursor].is_ascii_whitespace() {
            self.cursor += 1;
        }
    }

    fn consume_magic_number(&mut self) {
        let magic_number = self.consume_str(2);
        if magic_number != "P5" {
            panic!("Invalid magic number: {}", magic_number);
        }
    }

    fn into_remaining_bytes(self) -> Vec<u8> {
        self.bytes[self.cursor..].to_vec()
    }

    pub fn parse(bytes: Vec<u8>) -> PgmImage {
        let mut parser = Parser { bytes, cursor: 0 };

        parser.consume_magic_number();
        parser.consume_whitespace();
        let width = parser.consume_str_u16();
        parser.consume_whitespace();
        let height = parser.consume_str_u16();
        parser.consume_whitespace();
        let max_value = parser.consume_str_u16();
        parser.consume_whitespace();

        if max_value > u8::MAX as u16 {
            panic!("Unsupported max value: {}", max_value);
        }

        let data = parser.into_remaining_bytes();

        PgmImage {
            width: width as usize,
            height: height as usize,
            max_value,
            data,
        }
    }
}
