use ad_trait::AD;

pub fn get_range<T: AD>(range_start: T, range_stop: T, step_size: T) -> Vec<T> {
    let mut out_range = Vec::new();
    out_range.push(range_start);
    let mut last_added_val = range_start;

    while !( (range_stop - last_added_val).abs() < step_size ) {
        if range_stop > range_start {
            last_added_val = last_added_val + step_size;
        } else {
            last_added_val = last_added_val - step_size;
        }
        out_range.push(last_added_val);
    }

    out_range.push(range_stop);

    out_range
}