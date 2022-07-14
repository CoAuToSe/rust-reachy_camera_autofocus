#[derive(Clone, Copy, Debug)]
pub enum Side<T> {
    Left(T),
    Right(T),
}

impl<'a, T> Side<T> {
    pub fn inner(&self) -> &T {
        match self {
            Self::Left(to_return) => to_return,
            Self::Right(to_return) => to_return,
        }
    }
    pub const fn zoom(&self) -> &'a str {
        match self {
            Self::Left(_) => "X",
            Self::Right(_) => "Z",
        }
    }
    pub const fn focus(&self) -> &'a str {
        match self {
            Self::Left(_) => "Y",
            Self::Right(_) => "A",
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub enum SideOnly {
    Left,
    Right,
}
impl<'a> SideOnly {
    pub const fn zoom(&self) -> &'a str {
        match self {
            Self::Left => "X",
            Self::Right => "Z",
        }
    }
    pub const fn focus(&self) -> &'a str {
        match self {
            Self::Left => "Y",
            Self::Right => "A",
        }
    }
}

impl<T> From<Side<T>> for SideOnly {
    fn from(some: Side<T>) -> Self {
        match some {
            Side::Left(_) => SideOnly::Left,
            Side::Right(_) => SideOnly::Right,
        }
    }
}
impl<T> From<&Side<T>> for SideOnly {
    fn from(some: &Side<T>) -> Self {
        match some {
            Side::Left(_) => SideOnly::Left,
            Side::Right(_) => SideOnly::Right,
        }
    }
}

impl TryFrom<&str> for SideOnly {
    fn try_from(side: &str) -> std::result::Result<Self, ()> {
        match side {
            "left_eye" | "left_side" | "left" => Ok(Self::Left),
            "right_eye" | "right_side" | "right" => Ok(Self::Right),
            _ => Err(()),
        }
    }
    type Error = ();
}

impl TryFrom<String> for SideOnly {
    fn try_from(side: String) -> std::result::Result<Self, ()> {
        match &side as &str {
            "left_eye" | "left_side" | "left" => Ok(Self::Left),
            "right_eye" | "right_side" | "right" => Ok(Self::Right),
            _ => Err(()),
        }
    }
    type Error = ();
}

#[derive(Clone, Copy, Debug)]
pub enum ZoomLevel {
    In,
    Inter,
    Out,
}

impl TryFrom<&str> for ZoomLevel {
    fn try_from(zoom_level: &str) -> std::result::Result<Self, ()> {
        match zoom_level {
            "in" | "In" | "IN" => Ok(Self::In),
            "inter" | "Inter" | "INTER" => Ok(Self::Inter),
            "out" | "Out" | "OUT" => Ok(Self::Out),
            _ => Err(()),
        }
    }
    type Error = ();
}
impl TryFrom<String> for ZoomLevel {
    fn try_from(zoom_level: String) -> std::result::Result<Self, ()> {
        match &zoom_level as &str {
            "in" | "In" | "IN" => Ok(Self::In),
            "inter" | "Inter" | "INTER" => Ok(Self::Inter),
            "out" | "Out" | "OUT" => Ok(Self::Out),
            _ => Err(()),
        }
    }
    type Error = ();
}

#[derive(Clone, Copy, Debug)]
pub enum ZoomOrFocus<T> {
    Zoom(T),
    Focus(T),
}

impl<T> ZoomOrFocus<T> {
    pub fn value(&self) -> &T {
        match self {
            ZoomOrFocus::Zoom(value) => value,
            ZoomOrFocus::Focus(value) => value,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct ZoomFocus<T> {
    pub zoom: T,
    pub focus: T,
}

impl<'a, T> Side<ZoomFocus<T>> {
    pub fn zoom_value(&'a self) -> &'a T {
        match self {
            Self::Left(zoom_focus) => &zoom_focus.zoom,
            Self::Right(zoom_focus) => &zoom_focus.zoom,
        }
    }
    pub fn focus_value(&'a self) -> &'a T {
        match self {
            Self::Left(zoom_focus) => &zoom_focus.focus,
            Self::Right(zoom_focus) => &zoom_focus.focus,
        }
    }
}
impl<'a, T> Side<ZoomOrFocus<T>> {
    pub const fn identifier(&self) -> &str {
        match self {
            Self::Left(zoom_or_focus) => match zoom_or_focus {
                ZoomOrFocus::Zoom(_) => "X",
                ZoomOrFocus::Focus(_) => "Y",
            },
            Self::Right(zoom_or_focus) => match zoom_or_focus {
                ZoomOrFocus::Zoom(_) => "Z",
                ZoomOrFocus::Focus(_) => "A",
            },
        }
    }
    pub fn zoom_value(&'a self) -> &'a T {
        match self {
            Self::Left(zoom_or_focus) => &zoom_or_focus.value(),
            Self::Right(zoom_or_focus) => &zoom_or_focus.value(),
        }
    }
    pub fn focus_value(&'a self) -> &'a T {
        match self {
            Self::Left(zoom_or_focus) => &zoom_or_focus.value(),
            Self::Right(zoom_or_focus) => &zoom_or_focus.value(),
        }
    }
}

// impl From<Side<ZoomLevel>> for Side<ZoomFocus<isize>> {
//     fn from(side_zoom_level: Side<ZoomLevel>) -> Self {
//         match side_zoom_level {
//             Side::Left(zoom_level) => match zoom_level {
//                 ZoomLevel::In => Side::Left(ZoomFocus {
//                     zoom: 457,
//                     focus: 70,
//                 }),
//                 ZoomLevel::Inter => Side::Left(ZoomFocus {
//                     zoom: 270,
//                     focus: 331,
//                 }),
//                 ZoomLevel::Out => Side::Left(ZoomFocus {
//                     zoom: 60,
//                     focus: 455,
//                 }),
//             },
//             Side::Right(zoom_level) => match zoom_level {
//                 ZoomLevel::In => Side::Right(ZoomFocus {
//                     zoom: 457,
//                     focus: 42,
//                 }),
//                 ZoomLevel::Inter => Side::Right(ZoomFocus {
//                     zoom: 270,
//                     focus: 321,
//                 }),
//                 ZoomLevel::Out => Side::Right(ZoomFocus {
//                     zoom: 60,
//                     focus: 445,
//                 }),
//             },
//         }
//     }
// }
impl<T> Into<Side<ZoomFocus<T>>> for Side<ZoomLevel>
where
    T: From<isize>,
{
    fn into(self) -> Side<ZoomFocus<T>> {
        match self {
            Side::Left(zoom_level) => match zoom_level {
                ZoomLevel::In => Side::Left(ZoomFocus {
                    zoom: T::from(457),
                    focus: T::from(70),
                }),
                ZoomLevel::Inter => Side::Left(ZoomFocus {
                    zoom: T::from(270),
                    focus: T::from(331),
                }),
                ZoomLevel::Out => Side::Left(ZoomFocus {
                    zoom: T::from(60),
                    focus: T::from(455),
                }),
            },
            Side::Right(zoom_level) => match zoom_level {
                ZoomLevel::In => Side::Right(ZoomFocus {
                    zoom: T::from(457),
                    focus: T::from(42),
                }),
                ZoomLevel::Inter => Side::Right(ZoomFocus {
                    zoom: T::from(270),
                    focus: T::from(321),
                }),
                ZoomLevel::Out => Side::Right(ZoomFocus {
                    zoom: T::from(60),
                    focus: T::from(445),
                }),
            },
        }
    }
}

impl<T> Into<Vec<Side<ZoomOrFocus<T>>>> for Side<ZoomLevel>
where
    T: From<isize>,
{
    fn into(self) -> Vec<Side<ZoomOrFocus<T>>> {
        let temp: Side<ZoomFocus<T>> = self.into();
        temp.into()
    }
}

// impl<T> From<Side<ZoomFocus<T>>> for Vec<Side<ZoomOrFocus<T>>> {
//     fn from(some: Side<ZoomFocus<T>>) -> Self {
//         let mut vec = vec!();
//         match some {

//         }
//     }
// }

impl<T> Into<Vec<Side<ZoomOrFocus<T>>>> for Side<ZoomFocus<T>> {
    fn into(self) -> Vec<Side<ZoomOrFocus<T>>> {
        let mut vec = vec![];
        match self {
            Self::Left(zoom_focus) => {
                vec.push(Side::Left(ZoomOrFocus::Zoom(zoom_focus.zoom)));
                vec.push(Side::Left(ZoomOrFocus::Focus(zoom_focus.focus)));
            }
            Self::Right(zoom_focus) => {
                vec.push(Side::Right(ZoomOrFocus::Zoom(zoom_focus.zoom)));
                vec.push(Side::Right(ZoomOrFocus::Focus(zoom_focus.focus)));
            }
        }
        vec
    }
}

impl<T> Into<Side<ZoomFocus<T>>> for (SideOnly, ZoomFocus<T>) {
    fn into(self) -> Side<ZoomFocus<T>> {
        match self.0 {
            SideOnly::Left => Side::Left(self.1),
            SideOnly::Right => Side::Right(self.1),
        }
    }
}

// #[derive(Clone, Copy, Debug)]
// pub enum Side<T> {
//     Left(T),
//     Right(T),
// }

// impl<T> Side<T> {
//     pub fn inner(self) -> T {
//         match self {
//             Self::Left(to_return) => to_return,
//             Self::Right(to_return) => to_return,
//         }
//     }
// }

// #[derive(Clone, Copy, Debug)]
// pub enum SideOnly {
//     Left,
//     Right,
// }

// impl TryFrom<&str> for SideOnly {
//     fn try_from(side: &str) -> std::result::Result<Self, ()> {
//         match side {
//             "left_eye" | "left_side" | "left" => Ok(Self::Left),
//             "right_eye" | "right_side" | "right" => Ok(Self::Right),
//             _ => Err(()),
//         }
//     }
//     type Error = ();
// }

// impl TryFrom<String> for SideOnly {
//     fn try_from(side: String) -> std::result::Result<Self, ()> {
//         match &side as &str {
//             "left_eye" | "left_side" | "left" => Ok(Self::Left),
//             "right_eye" | "right_side" | "right" => Ok(Self::Right),
//             _ => Err(()),
//         }
//     }
//     type Error = ();
// }

// #[derive(Clone, Copy, Debug)]
// pub enum ZoomLevel {
//     In,
//     Inter,
//     Out,
// }
// impl TryFrom<&str> for ZoomLevel {
//     fn try_from(zoom_level: &str) -> std::result::Result<Self, ()> {
//         match zoom_level {
//             "in" | "In" | "IN" => Ok(Self::In),
//             "inter" | "Inter" | "INTER" => Ok(Self::Inter),
//             "out" | "Out" | "OUT" => Ok(Self::Out),
//             _ => Err(()),
//         }
//     }
//     type Error = ();
// }
// impl TryFrom<String> for ZoomLevel {
//     fn try_from(zoom_level: String) -> std::result::Result<Self, ()> {
//         match &zoom_level as &str {
//             "in" | "In" | "IN" => Ok(Self::In),
//             "inter" | "Inter" | "INTER" => Ok(Self::Inter),
//             "out" | "Out" | "OUT" => Ok(Self::Out),
//             _ => Err(()),
//         }
//     }
//     type Error = ();
// }

// #[derive(Clone, Copy, Debug)]
// pub enum ZoomOrFocus<T> {
//     Zoom(T),
//     Focus(T),
// }

// #[derive(Clone, Copy, Debug)]
// pub struct ZoomFocus<T> {
//     pub zoom: T,
//     pub focus: T,
// }

// impl<'a, T> Side<ZoomFocus<T>> {
//     pub fn zoom_value(&'a mut self) -> &'a T {
//         match self {
//             Self::Left(zoom_focus) => &mut zoom_focus.zoom,
//             Self::Right(zoom_focus) => &mut zoom_focus.zoom,
//         }
//     }
//     pub fn focus_value(&'a mut self) -> &'a T {
//         match self {
//             Self::Left(zoom_focus) => &mut zoom_focus.focus,
//             Self::Right(zoom_focus) => &mut zoom_focus.focus,
//         }
//     }
// }
