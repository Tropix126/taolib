/**
 * @file src/taolib/threading.h
 * @author ModEngineer
 *
 * Various various helpers for creating and working with multithreaded
 * applications on vexcode. These helpers assist in various tasks that
 * would normally be difficult to achieve with the vexcode-provided
 * vex::thread class, such as starting a thread from a non-static class
 * member, or passing arguments to threads.
 */

#pragma once

#include <tuple>
#include <utility>

#include "v5_cpp.h"

namespace tao::threading {
namespace internal {

// Implementation of several missing standard library clstructs and functions to allow for `apply_`
template <std::size_t...>
struct index_sequence_ {};

template <std::size_t N, std::size_t... Next>
struct index_sequence_helper_ : public index_sequence_helper_<N - 1U, N - 1U, Next...> {};

template <std::size_t... Next>
struct index_sequence_helper_<0U, Next...> {
  using type = index_sequence_<Next...>;
};

template <std::size_t N>
using make_index_sequence_ = typename index_sequence_helper_<N>::type;

template <typename Fn, typename Tuple, size_t... I>
void apply_(Fn* fn, Tuple t, index_sequence_<I...>) {
  (*fn)(std::get<I>(t)...);
}

template <typename Fn, typename... Args>
void wrap_func_(void* func_and_args_void) {
  // Convert the void pointer to a pointer to the tuple of the function and arguments
  auto* func_and_args = (std::tuple<Fn, std::tuple<Args...>, bool*>*)func_and_args_void;
  // Mark the thread as initialized
  *(std::get<2>(*func_and_args)) = true;
  // Run the function with the arguments
  apply_(std::get<0>(*func_and_args), std::get<1>(*func_and_args), make_index_sequence_<sizeof...(Args)>{});
  delete func_and_args;
}

}

/**
 * Creates a vex::thread that runs a given function with arguments
 * @param Ret return type of `fn`
 * @param Args types of arguments to `fn`
 * @param fn the target function
 * @param args the arguments to `fn`
 * @return vex::thread that is running `fn`
 */
template <typename Ret, typename... Args>
vex::thread make_thread(Ret (*fn)(Args...), Args... args) {
  // Create a boolean value that gets set to true by the thread so that the thread gets initialized before this function exits; vex STL is buggy
  bool fake_promise = false;
  // Put the function pointer and all arguments into a tuple
  auto* func_and_args = new std::tuple<decltype(fn), std::tuple<Args...>, bool*>(fn, std::forward_as_tuple(args...), &fake_promise);
  // Create a thread that runs the function wrapper
  vex::thread internal_thread = vex::thread(internal::wrap_func_<decltype(fn), Args...>, (void*)func_and_args);
  // Wait for thread initialization
  while (!fake_promise) {
	vex::task::sleep(1);
  }
  return internal_thread;
}

template <typename Cls, typename Ret, typename... Args>
/**
 * Wraps a member function to allow calling it like a static function
 * @tparam Cls type of `cls_instance`
 * @tparam Ret return type `cls_fn`
 * @tparam Args types of arguments to `cls_fn`
 * @param cls_instance pointer to instance of class to run `cls_fn` on
 * @param cls_fn pointer to function callable from `cls_instance`; i.e., `&Cls::some_function`
 * @param args arguments to `cls_fn`
 * @return whatever value `cls_fn(args...)` returns
 */
Ret static_proxy(Cls* cls_instance, Ret (Cls::*cls_fn)(Args...), Args... args) {
  return (cls_instance->*cls_fn)(std::forward<Args>(args)...);
}

/**
 * Creates a vex::thread that runs a member function with arguments
 * @tparam Cls type of `cls_instance`
 * @tparam Ret return type of `cls_fn`
 * @tparam Args types of arguments to `cls_fn`
 * @param cls_instance pointer to instance of class to run `cls_fn` on
 * @param cls_fn pointer to function callable from `cls_instance`; i.e., `&Cls::some_function`
 * @param args arguments to `cls_fn`
 * @return vex::thread that is running `cls_fn`
 */
template <typename Cls, typename Ret, typename... Args>
vex::thread make_member_thread(Cls* cls_instance, Ret (Cls::*cls_fn)(Args...), Args... args) {
  return make_thread(static_proxy<Cls, Ret, Args...>, cls_instance, cls_fn, std::forward<Args>(args)...);
}
}