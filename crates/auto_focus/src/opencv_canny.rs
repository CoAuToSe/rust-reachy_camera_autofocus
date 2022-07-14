use sensor_msgs::msg::CompressedImage;
use std::fmt::Debug;

///Return the shaprness of im through canny edge dectection algorithm.
///
///Args:
///    im: image used in canny edge detection algorithm
pub fn canny_sharpness_function<T>(im: &CompressedImage) -> T
where
    T: 'static
        // + From<isize>
        + std::convert::From<f64>
        + std::convert::From<i32>
        + std::ops::Mul<Output = T>
        + Copy
        + Debug
        + Default
        + std::marker::Send
        + std::ops::AddAssign
        + std::ops::Add<Output = T>
        + std::cmp::PartialOrd
        + std::ops::Sub<Output = T>
        + std::ops::Div<Output = T>,
    f64: std::convert::From<T>,
{
    use opencv as cv;
    let src = cv::core::Mat::from_slice::<u8>(im.data.as_ref()).unwrap();
    let img = cv::imgcodecs::imdecode(&src, cv::imgcodecs::IMREAD_GRAYSCALE).unwrap();
    let mut dst_img = cv::core::Mat::default();
    cv::imgproc::canny(&img.clone(), &mut dst_img, 50., 100., 3, false).unwrap();
    let mut sum: usize = 0;
    for e in cv::prelude::MatTraitConstManual::data_bytes(&dst_img)
        .unwrap()
        .iter()
    {
        sum += *e as usize
    }
    // ezprint!(
    //     &dst_img,
    //     sum,
    //     &cv::prelude::MatTraitConstManual::size(&dst_img)
    // );
    let tem = cv::prelude::MatTraitConstManual::size(&dst_img)
        .ok()
        .unwrap();
    return T::from(T::from(sum as f64) / T::from(tem.width * tem.height));
}
