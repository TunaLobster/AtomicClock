template<unsigned End>
class usmall {
public:
    // implicit conversion constructor, making sure the value is in the range [0, End]
    usmall(unsigned char v = 0) : data(v % (End + 1)) {}

    // implicit user defined conversion operator
    operator unsigned char () const { return data; }

    // pre increment and decrement operators
    usmall& operator++() { if(data==End) data = 0; else ++data; return *this; }
    usmall& operator--() { if(data==0) data = End; else --data; return *this; }

    // post increment and decrement operators
    usmall operator++(int) { usmall rv(*this); ++*this; return rv; }
    usmall operator--(int) { usmall rv(*this); --*this; return rv; }

    // common arithmetic operators
    usmall& operator+=(const usmall& rhs) {
        data = (data + rhs.data) % (End + 1);
        return *this;
    }
    
    usmall& operator-=(const usmall& rhs) {
        if(data < rhs.data) data += (End + 1);
        data -= rhs.data;
        return *this;
    }

    usmall& operator*=(const usmall& rhs) {
        data = (data * rhs.data) % (End + 1);
        return *this;
    }
    
    usmall& operator/=(const usmall& rhs) {
        data /= rhs.data;
        return *this;
    }

private:
    unsigned int data;
};

// free arithmetic functions
template<unsigned End>
auto operator+(const usmall<End>& lhs, const usmall<End>& rhs) {
    usmall rv(lhs);
    rv += rhs;
    return rv;
}

template<unsigned End>
auto operator-(const usmall<End>& lhs, const usmall<End>& rhs) {
    usmall rv(lhs);
    rv -= rhs;
    return rv;
}

template<unsigned End>
auto operator*(const usmall<End>& lhs, const usmall<End>& rhs) {
    usmall rv(lhs);
    rv *= rhs;
    return rv;
}

template<unsigned End>
auto operator/(const usmall<End>& lhs, const usmall<End>& rhs) {
    usmall rv(lhs);
    rv /= rhs;
    return rv;
}