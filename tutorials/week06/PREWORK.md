Week 6 - PREWORK
=========================

Before we start developing code that uses threading, we will have a look at an implementation of threading that allows us to understand basic concepts. 

Simple_Threading
-----------------------------------------

In this example we will create a number of threads that will try to access a shared resource (a vector, that is passed by reference). This example is simplistic in nature, but it shows that concurrency (threads that run in parallel), could have unpredictable behaviour if not managed properly. You can run this code several times and it may finish, or more likely it will fail with a segmentation fault.

The code you have been supplied has four threads (`t_0` to `t_3`) that then access a vector `vec` and add values to it. When we create a thread the syntax used is:
```
std::thread t_0(addNum,id++,ref(numVec),ref(numMutex));
```
Here `t_0` is the thread name, while there are four arguments
1. first argument is the function that is run by the thread, here it is `addNum`. 
2. second argument is an integer that we increment (this allows us to assign a number to the thread)
3. third argument is passed by reference (hence the `ref`) and it is `numVec`
4. fourth argument is passed by reference (hence the `ref`) and it is `numMutex`

The arguments need to match the function declaration, and if we have a look at the declaration of `addNum` is:
```
void addNum(int id, std::vector<int>& vec, std::mutex &numMutex) {
```
Which matches the arguments specified when starting the thread.


The code may run, and present numbers in somewhat random order, or it  will terminate with an error:
```
double free or corruption (fasttop)
Aborted (core dumped)
```

Attempt the following:

* Comment out the four lines doing the join of threads, recompile code, what occurs? 
* Return the four lines doing the join of threads, recompile, any changes?
* Lock and unlock the [mutex](https://en.cppreference.com/w/cpp/thread/mutex), is there a change?
* Use a [scoped lock](https://en.cppreference.com/w/cpp/thread/scoped_lock) or [lock guard](https://en.cppreference.com/w/cpp/thread/lock_guard) instead?
* Consider why a mutex is needed, why do we share the same mutex, is there a segfault occurring? 