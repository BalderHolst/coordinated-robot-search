use std::{
    sync::mpsc::{self, Receiver, Sender},
    thread,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct DataId(usize);

pub struct ThreadPool<A, B>
where
    A: Send + 'static,
    B: Send + 'static,
{
    threads: usize,
    input_channels: Vec<Sender<Option<(DataId, A)>>>,
    output_channel: Receiver<(DataId, B)>,
    handles: Vec<thread::JoinHandle<()>>,
}

impl<A, B> ThreadPool<A, B>
where
    A: Send + 'static,
    B: Send + 'static,
{
    pub fn new(threads: usize, f: fn(A) -> B) -> ThreadPool<A, B> {
        let (output_tx, output_rx) = mpsc::channel();

        let (input_channels, handles): (Vec<_>, Vec<_>) = (0..threads)
            .map(|_| {
                let (input_tx, input_rx) = mpsc::channel();
                let handle = {
                    let output_tx = output_tx.clone();
                    thread::spawn(move || {
                        while let Ok(Some((id, input))) = input_rx.recv() {
                            let output = f(input);
                            output_tx.send((id, output)).unwrap();
                        }
                    })
                };
                (input_tx, handle)
            })
            .unzip();

        ThreadPool {
            threads,
            input_channels,
            output_channel: output_rx,
            handles,
        }
    }

    pub fn process(&self, input: Vec<A>) -> Vec<B> {
        let len = input.len();
        for (n, item) in input.into_iter().enumerate() {
            let id = DataId(n);
            let channel = &self.input_channels[n % self.threads];
            channel.send(Some((id, item))).unwrap();
        }
        let mut results = Vec::with_capacity(len);
        for _ in 0..len {
            results.push(self.output_channel.recv().unwrap());
        }
        results.sort_by(|(ida, _), (idb, _)| ida.cmp(idb));
        results.into_iter().map(|(_, item)| item).collect()
    }
}

impl<A, B> Drop for ThreadPool<A, B>
where
    A: Send + 'static,
    B: Send + 'static,
{
    fn drop(&mut self) {
        for input_channel in self.input_channels.drain(..) {
            // Send None to signal the thread to stop their internal loop
            input_channel.send(None).unwrap();
        }
        for handle in self.handles.drain(..) {
            // Wait for all threads to finish
            handle.join().unwrap();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_thread_pool() {
        let pool = ThreadPool::new(4, |x: i32| x * 2);
        let input = vec![1, 2, 3, 4, 5, 6, 7, 8, 9, 10];
        let output = pool.process(input);
        assert_eq!(output, vec![2, 4, 6, 8, 10, 12, 14, 16, 18, 20]);

        let pool = ThreadPool::new(1, |x: i32| x * x);
        let input = vec![1, 2, 3, 4];
        let output = pool.process(input);
        assert_eq!(output, vec![1, 4, 9, 16]);

        let pool = ThreadPool::new(10, |(a, b): (i32, i32)| a + b);
        let input = vec![(1, 2), (3, 4), (5, 6), (7, 8)];
        let output = pool.process(input);
        assert_eq!(output, vec![3, 7, 11, 15]);
    }
}
