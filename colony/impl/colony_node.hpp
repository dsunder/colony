// cxx@kayari.org

#ifndef COLONY_IMPL_NODE_HPP
#define COLONY_IMPL_NODE_HPP

#include <climits>
#include <cstdint>
#include <type_traits>

namespace Std { namespace Impl {

namespace Bits {

//------------------------------------------------------------------------------
// int count_trailing_zeros(unsigned x)
// int count_trailing_zeros(unsigned long x)
// int count_trailing_zeros(unsigned long long x)
//
// Returns the number of trailing 0-bits in x, starting at the least significant
// bit position. If x is 0, sizeof(Unsigned)*CHAR_BIT.
//------------------------------------------------------------------------------
inline __attribute__((always_inline)) constexpr
int count_trailing_zeros(unsigned x) noexcept
{ return x ? __builtin_ctz(x) : sizeof(unsigned) * CHAR_BIT; }

inline __attribute__((always_inline)) constexpr
int count_trailing_zeros(unsigned long x) noexcept
{ return x ? __builtin_ctzl(x) : sizeof(unsigned long) * CHAR_BIT; }

inline __attribute__((always_inline)) constexpr
int count_trailing_zeros(unsigned long long x) noexcept
{ return x ? __builtin_ctzll(x) : sizeof(unsigned long long) * CHAR_BIT; }
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// int popcount(unsigned x)
// int popcount(unsigned long x)
// int popcount(unsigned long long x)
//
// Return the number of 1-bits in x.
//------------------------------------------------------------------------------
inline __attribute__((always_inline)) constexpr
int popcount(unsigned x) const  noexcept
{ return __builtin_popcount(x); }

inline __attribute__((always_inline)) constexpr
int popcount(unsigned long x) const  noexcept
{ return __builtin_popcountl(x); }

inline __attribute__((always_inline)) constexpr
int popcount(unsigned long long x) const  noexcept
{ return __builtin_popcountll(x); }
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Unsigned complement(unsigned x)
// Unsigned complement(unsigned long x)
// Unsigned complement(unsigned long long x)
//
// Return the complement of x.
// number of 1-bits in x.
//------------------------------------------------------------------------------
inline __attribute__((always_inline)) constexpr
unsigned complement(unsigned x) noexcept
{ return ~x; }

inline __attribute__((always_inline)) constexpr
unsigned long complement(unsigned long x) noexcept
{ return ~x; }

inline __attribute__((always_inline)) constexpr
unsigned long long complement(unsigned long long x) noexcept
{ return ~x; }
//------------------------------------------------------------------------------
} //namespace Bits

//------------------------------------------------------------------------------
// atomic bitset for colony
//------------------------------------------------------------------------------
template <class Unsigned, bool IsAtomic = true>
class colony_bitset_impl
{
  static_assert(  std::is_same_v<Unsigned, unsigned>
               || std::is_same_v<Unsigned, unsigned long>
               || std::is_same_v<Unsigned, unsigned long long>
               || std::is_same_v<Unsigned, size_t>
               );

  Unsigned& m_ref;
public:

  using value_type = Unsigned;
  static constexpr bool is_atomic      = IsAtomic;
  static constexpr value_type capacity = value_type{sizeof(value_type)*CHAR_BIT};
  static constexpr value_type zero     = value_type{0};
  static constexpr value_type one      = value_type{1};
  static constexpr value_type all      = Bits::complement(zero);

  colony_bitset_impl()                                     = delete;
  colony_bitset_impl(colony_bitset_impl const&)            = delete;
  colony_bitset_impl(colony_bitset_impl &&)                = delete;
  colony_bitset_impl& operator=(colony_bitset_impl const&) = delete;
  colony_bitset_impl& operator=(colony_bitset_impl &&)     = delete;

  ~colony_bitset_impl() = default;


  // Returns an Unsigned with only the lower n bits turned on.
  inline __attribute__((always_inline)) constexpr
  static value_type create_mask(int n) noexcept
  {
    return (one << n) - one;

  }

  // init from reference
  inline __attribute__((always_inline))
  explicit colony_bitset_impl(value_type& v) noexcept : m_ref{v} {}

  inline __attribute__((always_inline))
  value_type load() const noexcept
  {
    if constexpr (is_atomic) {
      // TODO use atomic_ref.load
      return __atomic_load_n( &m_ref
                            , __ATOMIC_ACQUIRE
                            );
    } else {
      return m_ref;
    }
  }

  inline __attribute__((always_inline))
  value_type store(value_type val) const noexcept
  {
    if constexpr (is_atomic) {
      // TODO use atomic_ref.store
      __atomic_store_n( &m_ref
                      , val
                      , __ATOMIC_RELEASE
                      );
    } else {
      m_ref = val;
    }
  }

  // Clear the bits in m_ref that are on in val.
  // Returns a value_type with the bits this operation clear in m_ref turned on.
  inline __attribute__((always_inline))
  value_type clear_bits(value_type val) noexcept
  {
    if constexpr (is_atomic) {
      // TODO use atomic_ref.fetch_and
      value_type res = __atomic_fetch_and( &m_ref
                                         , Bits::complement(val)
                                         , __ATOMIC_ACQ_REL
                                         );
      return res & val;

    } else {
      value_type res = m_ref;
      m_ref = m_ref & Bits::complement(val);
      return res & val;
    }
  }

  // Set the bits in m_ref that are on in val.
  // Returns a value_type with the bits this operation set in m_ref turned on.
  inline __attribute__((always_inline))
  value_type set_bits(value_type val) noexcept
  {
    if constexpr (is_atomic) {
      // TODO use atomic_ref.fetch_or
      value_type res = __atomic_fetch_or( &m_ref
                                        , val
                                        , __ATOMIC_ACQ_REL
                                        );
      return Bits::complement(res) & val;
    } else {
      value_type res = m_ref;
      m_ref = m_ref | val;
      return Bits::complement(res) & val;
    }
  }

  // Used to conditionally set the bitset
  inline __attribute__((always_inline))
  bool try_set( value_type& expected, value_type desired ) noexcept
  {
    if constexpr (is_atomic) {
      // TODO use atomic_ref.compare_exchange_strong
      return __atomic_compare_exchange_n( &m_ref
                                        , &expected
                                        , desired
                                        , false // strong --> no spurious failures
                                        , __ATOMIC_ACQ_REL
                                        , __ATOMIC_ACQUIRE
                                        );
    } else {
      return m_ref == zero
           ? (m_ref = all , true)
           : false
           ;
    }
  }

  // Returns a currently set bit greater than or equal to i or capacity if
  // there are no bits currently set greater than or equal to i
  inline __attribute__((always_inline))
  int find_set_bit(int i = 0) const noexcept
  {
    return Bits::count_trailing_zeros(load()
         & Bits::complement(create_mask<value_type>(i)));
  }

  // Returns a currently unset bit greater than or equal to i or capacity if
  // there are no bits currently unset greater than or equal to i
  inline __attribute__((always_inline))
  int find_unset_bit(int i = 0) const noexcept
  {
    return Bits::count_trailing_zeros(Bits::complement(load())
         & Bits::complement(create_mask(i)));
  }

  inline __attribute__((always_inline))
  int size() const noexcept
  { return Bits::popcount(load()); }
};

template <typename Unsigned>
using colony_bitset_ref = colony_bitset_impl<Unsigned, false>;

template <typename Unsigned>
using atomic_colony_bitset_ref = colony_bitset_impl<Unsigned, true>;
//------------------------------------------------------------------------------

enum NodeState : uint64_t
{
  UNLINK_THIS_NODE   = 1; // Lazily mark this node to be unlinked
  UNLINK_NEXT_NODE   = 2; // Unlink the next node in the linked list and place it on
                        // the sentinal's free list
  INACTIVE_NODE      = 4; // The node in in the sentinal's free list
  DRAINING_SENTINAL  = 8; // The colony is being drained

};


template <bool IsAtomic = true>
class colony_node_ptr_impl
{
  uint64_t m_ref;

  inline __attribute__((always_inline))
  uint64_t compare_exchange_strong( uint64_t& expected
                                  , uint64_t  desired
                                  ) noexcept
  {
    if constexpr (is_atomic) {
      // TODO use atomic_ref.compare_exchange_strong
      return __atomic_compare_exchange_n( &m_ref
                                        , &expected
                                        , desired
                                        , false // strong --> no spurious failures
                                        , __ATOMIC_ACQ_REL
                                        , __ATOMIC_ACQUIRE
                                        );
    } else {
      return m_ref == expected
           ? (m_ref    = desired , true)
           : (expected = m_ref   , false)
           ;
    }
  }

public:
  static constexpr bool is_atomic = IsAtomic;


  // Atomic bitsets are not copyable, moveable, or assignable
  colony_node_ptr_impl() noexcept                              = delete;
  colony_node_ptr_impl(colony_node_ptr_impl const&)            = delete;
  colony_node_ptr_impl(colony_node_ptr_impl &&)                = delete;
  colony_node_ptr_impl& operator=(colony_node_ptr_impl const&) = delete;
  colony_node_ptr_impl& operator=(colony_node_ptr_impl &&)     = delete;

  ~colony_node_ptr_impl() = default;

  inline __attribute__((always_inline))
  static bool next_marked_unlink( uint64_t next_next ) noexcept
  { return bool(next_next & UNLINK_THIS_NODE); }

  // init from reference
  inline __attribute__((always_inline))
  explicit colony_node_ptr_impl(uint64_t& v) noexcept : m_ref{v} {}

  inline __attribute__((always_inline))
  uint64_t load() const noexcept
  {
    if constexpr (is_atomic) {
      // TODO use atomic_ref.load
      return __atomic_load_n( &m_ref
                            , __ATOMIC_ACQUIRE
                            );
    } else {
      return m_ref;
    }
  }

  inline __attribute__((always_inline))
  void store(uint64_t val) noexcept
  {
    if constexpr (is_atomic) {
      // TODO use atomic_ref.store
      __atomic_store_n( &m_ref
                      , val
                      , __ATOMIC_RELEASE
                      );
    } else {
      m_ref = val;
    }
  }

  inline __attribute__((always_inline))
  bool try_mark_unlink_this() noexcept
  {
    auto expected = load();
    if ( !(expected & UNLINK_THIS_NODE) ) {
      return compare_exchange_strong( expected, (expected | UNLINK_THIS_NODE));
    }
    return false;
  }

  inline __attribute__((always_inline))
  bool try_unlink_next() noexcept
  {
    auto expected = load();
    if ( !(expected & UNLINK_NEXT_NODE) ) {
      return compare_exchange_strong( expected, (expected | UNLINK_NEXT_NODE));
    }
    return false;
  }

  inline __attribute__((always_inline))
  bool try_set_inactive() noexcept
  {
    auto expected = load();
    if ( !(expected & INACTIVE_NODE) ) {
      return compare_exchange_strong( expected, (expected | INACTIVE_NODE));
    }
    return false;
  }
};


class alignas(64) colony_node
{
  // ptr to next node
  // or to the sentinal if the node in on the free list
  // node state = (next_ptr & 63)
  uint64_t next_ptr;

  // ptr to the sentinal
  // or next node on free list if inactive
  // free state = (next_ptr & 63)
  uint64_t free_list_ptr;

  // bitset indicating if an index is used
  uint64_t used_bitset;

  // bitset indicating if an index is valid
  uint64_t valid_bitset;

  // ptr to the sentinal node
  // if the node is a sentinal, ptr to the next sentinal
  // bitset_size = (sentinal_list_ptr & 63) + 1
  uint64_t sentinal_list_ptr;

  // ptr to the head node
  // the bitset size is stored in the the lower 5 bits
  // node_type = (head_ptr & 63)
  uint64_t head_ptr;


};


}} // namespace Std::Impl

#endif // COLONY_IMPL_NODE_HPP
