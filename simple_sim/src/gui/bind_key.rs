#[macro_export]
macro_rules! bind {
    ($input:expr; $bind:tt, $($key:expr),+ => $e:expr) => {{
        if $($input.$bind($key))|+ {
            $e;
        }
    }}
}

#[macro_export]
macro_rules! bind_down    {
    ($input:expr; $($key:expr),+ => $e:expr) => {
        crate::bind!($input; key_down, $($key),+ => $e)
    }
}

#[macro_export]
macro_rules! bind_pressed {
    ($input:expr; $($key:expr),+ => $e:expr) => {
        crate::bind!($input; key_pressed, $($key),+ => $e)
    }
}
