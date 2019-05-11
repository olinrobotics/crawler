#!/usr/bin/env python

def mapPrecise(x, inMin, inMax, outMin, outMax):
  # Emulates Arduino map() function, but uses floats for precision
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;

def cmd2msg(c_in, c_range, m_range):
    # Given command, command range, and msg range, returns corresponding msg
    m_out = 0

    # Unpack range values for clarity
    c_low = c_range[0]
    c_mid = c_range[1]
    c_hi = c_range[2]
    m_low = m_range[0]
    m_mid = m_range[1]
    m_hi = m_range[2]

    # Convert from input command to output message
    if c_low < c_hi:  # Ensure correct direction of comparison - is c_high < c_low?
        if c_in < c_mid: m_out = mapPrecise(c_in, c_low, c_mid, m_low, m_mid)
        elif c_in > c_mid: m_out = mapPrecise(c_in, c_mid, c_hi, m_mid, m_hi)
        else: m_out = m_mid
    elif c_low > c_hi:
        if c_in > c_mid: m_out = mapPrecise(c_in, c_low, c_mid, m_low, m_mid)
        elif c_in < c_mid: m_out = mapPrecise(c_in, c_mid, c_hi, m_mid, m_hi)
        else: m_out = m_mid

    return m_out

def msg2cmd(m_in, m_range, c_range):
    pass
