#include <iostream>
#include <array> // so that the array can be passed by value
#include <iomanip>
#include <vector> // For DriveableUnit list
#include <sstream> // Needed for stringstream
#include <map>          // For mnemonic/register mapping
#include <charconv>     // For std::from_chars (C++17)
#include <algorithm>    // For std::copy, std::transform, std::find_if

/**
 * @brief Utility class for converting between integer representations and bit arrays,
 *        and for printing/formatting bit arrays.
 *
 * This class provides static methods to:
 * 1. Convert unsigned integers to std::array<unsigned int, N> where each element
 *    is a bit (0 or 1), stored in Little-Endian format (LSB at index 0).
 * 2. Convert std::array<unsigned int, N> (Little-Endian) back to unsigned integers,
 *    unsigned long long, and signed integers (handling two's complement).
 * 3. Extract and insert bit fields between arrays.
 * 4. Print bit arrays in binary (MSB first) and hexadecimal formats.
 *
 * All methods are static, so no instantiation of the Convert class is needed.
 */
class Convert {
public:
    // --- Integer to Array Conversion Functions ---
    // These functions convert the N least significant bits (LSBs) of an unsigned integer
    // into a std::array of N unsigned integers, where each element is either 0 or 1.
    // The bits are stored in Little-Endian order: bit 0 (LSB) of the value goes to array index 0.

    /**
     * @brief Converts the 24 LSBs of an unsigned integer to a 24-bit array.
     * @param value The input unsigned integer.
     * @return A std::array<unsigned int, 24> representing the 24 LSBs.
     *         result[i] holds bit i of the input value.
     */
    static std::array<unsigned int, 24> ToArray(unsigned int value) {
        std::array<unsigned int, 24> result;
        // Extracts the 24 least significant bits from 'value'.
        // Bit 'i' from 'value' is stored at result[i].
        for (int i = 0; i < 24; ++i) {
            result[i] = (value >> i) & 1; // Shift bit 'i' to position 0 and mask
        }
        return result;
    }

    /**
     * @brief Converts the 22 LSBs of an unsigned integer to a 22-bit array.
     * @param value The input unsigned integer.
     * @return A std::array<unsigned int, 22> representing the 22 LSBs.
     */
    static std::array<unsigned int, 22> ToArray22Bit(unsigned int value) {
        std::array<unsigned int, 22> result;
        // Extracts the 22 least significant bits.
        for (int i = 0; i < 22; ++i) {
            result[i] = (value >> i) & 1;
        }
        return result;
    }

    /**
     * @brief Converts the 8 LSBs of an unsigned integer to an 8-bit array.
     * @param value The input unsigned integer.
     * @return A std::array<unsigned int, 8> representing the 8 LSBs.
     */
    static std::array<unsigned int, 8> ToArray8Bit(unsigned int value) {
        std::array<unsigned int, 8> result;
        // Extracts the 8 least significant bits.
        for (int i = 0; i < 8; ++i) {
            result[i] = (value >> i) & 1;
        }
        return result;
    }

    /**
     * @brief Converts the 2 LSBs of an unsigned integer to a 2-bit array.
     * @param value The input unsigned integer.
     * @return A std::array<unsigned int, 2> representing the 2 LSBs.
     */
    static std::array<unsigned int, 2> ToArray2Bit(unsigned int value) {
        std::array<unsigned int, 2> result;
        // Extracts the 2 least significant bits.
        for (int i = 0; i < 2; ++i) {
            result[i] = (value >> i) & 1;
        }
        return result;
    }

    /**
     * @brief Converts the LSB of an unsigned integer to a 1-bit array.
     * @param value The input unsigned integer.
     * @return A std::array<unsigned int, 1> representing the LSB.
     */
    static std::array<unsigned int, 1> ToArray1Bit(unsigned int value) {
        std::array<unsigned int, 1> result;
        // Extracts the least significant bit (bit 0).
        result[0] = value & 1;
        return result;
    }

    /**
     * @brief Converts the 4 LSBs of an unsigned integer to a 4-bit array.
     * @param value The input unsigned integer.
     * @return A std::array<unsigned int, 4> representing the 4 LSBs.
     */
    static std::array<unsigned int, 4> ToArray4Bit(unsigned int value) {
        std::array<unsigned int, 4> result;
        // Extracts the 4 least significant bits.
        for (int i = 0; i < 4; ++i) {
            result[i] = (value >> i) & 1;
        }
        return result;
    }

    // --- Array to Integer Conversion Functions ---
    // These functions interpret a std::array of bits (Little-Endian: LSB at index 0)
    // and convert it to an integer type.

    /**
     * @brief Converts an N-bit array (LSB at index 0) to an unsigned integer.
     * @tparam N The number of bits in the input array.
     * @param arr The input N-bit array, where arr[i] is bit i.
     * @return The corresponding unsigned integer value.
     * @note If N is larger than the number of bits in an `unsigned int`,
     *       higher-order bits from the array are ignored (truncation).
     */
    template<size_t N>
    static unsigned int ArrayToUInt(const std::array<unsigned int, N>& arr) {
        unsigned int val = 0;
        // Limit processing to the minimum of N or the number of bits in an unsigned int
        // to prevent overflow and undefined behavior with shifts.
        size_t bits_to_process = std::min(N, (size_t)sizeof(unsigned int) * 8);
        for (size_t i = 0; i < bits_to_process; ++i) {
            // This check is technically redundant if bits_to_process is derived from N,
            // but good for safety if N could somehow be less than bits_to_process conceptually.
            if (i < N) {
                // Set bit 'i' of 'val' if arr[i] is 1.
                // arr[i] & 1 ensures we only consider the LSB of arr[i] (should be 0 or 1).
                val |= ((arr[i] & 1) << i);
            }
        }
        return val;
    }

    /**
     * @brief Converts a 24-bit array (LSB at index 0) to an unsigned integer.
     * @param array The input 24-bit array.
     * @return The corresponding unsigned integer value.
     */
    static unsigned int ArrayToUInt24(const std::array<unsigned int, 24>& array) {
        // Delegates to the templated ArrayToUInt function.
        return ArrayToUInt<24>(array);
    }

    /**
     * @brief Converts an N-bit array (LSB at index 0) to an unsigned long long.
     * @tparam N The number of bits in the input array.
     * @param arr The input N-bit array.
     * @return The corresponding unsigned long long value.
     * @note If N is larger than the number of bits in an `unsigned long long`,
     *       higher-order bits from the array are ignored.
     */
    template<size_t N>
    static unsigned long long ArrayToULongLong(const std::array<unsigned int, N>& arr) {
        unsigned long long val = 0;
        size_t bits_to_process = std::min(N, (size_t)sizeof(unsigned long long) * 8);
        for (size_t i = 0; i < bits_to_process; ++i) {
            if (i < N) {
                // Set bit 'i' of 'val'.
                val |= (static_cast<unsigned long long>(arr[i] & 1) << i);
            }
        }
        return val;
    }

    /**
     * @brief Converts an N-bit array (LSB at index 0) to a signed integer.
     * Handles two's complement representation. The MSB of the N-bit number
     * is arr[N-1].
     * @tparam N The number of bits in the input array.
     * @param arr The input N-bit array.
     * @return The corresponding signed integer value.
     * @note If N is larger than the number of bits in an `int`, truncation may occur.
     *       A warning is printed to stderr in this case.
     */
    template<size_t N>
    static int ArrayToInt(const std::array<unsigned int, N>& arr) {
        if (N == 0) {
            return 0; // Or throw an error, depending on desired behavior for empty array.
        }
        // Warn if N exceeds the capacity of a signed int, as sign extension logic
        // and representation might not be perfect.
        if (N > (sizeof(int) * 8)) {
            std::cerr << "Warning: ArrayToInt called with N=" << N
                << " which is larger than the number of bits in an int. Result may be truncated." << std::endl;
        }

        // First, get the value as if it were unsigned.
        unsigned int uval = ArrayToUInt<N>(arr);

        // Determine if the N-bit number is negative by checking its MSB (arr[N-1]).
        // The array is LSB-first, so index N-1 is the MSB of the N-bit number.
        bool is_negative = (N > 0) && (arr[N - 1] == 1);

        if (is_negative && N <= (sizeof(int) * 8)) {
            // If negative and fits within an int, perform sign extension.
            unsigned int sign_extend_mask = 0;
            if (N < (sizeof(int) * 8)) { // Avoid undefined shift if N equals bit size of int
                // Create a mask like 11...1100...0 (N zeros at the LSB end).
                // This sets all bits from position N up to the MSB of 'unsigned int'.
                sign_extend_mask = ~((1U << N) - 1U);
            }
            // If N == sizeof(int)*8, uval already has the correct bit pattern for a negative int.
            // The static_cast<int> will interpret it correctly.
            return static_cast<int>(uval | sign_extend_mask);
        }
        else if (is_negative && N > (sizeof(int) * 8)) {
            // If negative but N is too large for int, the value is truncated.
            // The sign bit of the original N-bit number might not be the sign bit of the resulting int.
            // Return the truncated unsigned value cast to int.
            return static_cast<int>(uval);
        }
        else {
            // Positive number or N=0.
            return static_cast<int>(uval);
        }
    }


    // --- Field Extraction and Insertion ---
    // These helpers operate on arrays of bits, assuming LSB-first indexing.

    /**
     * @brief Extracts a sub-array (field) from a larger source array.
     * Assumes LSB-first indexing for both source and destination.
     * @tparam FieldSize The number of bits in the field to extract.
     * @tparam TotalSize The total number of bits in the source array.
     * @param source The source array from which to extract.
     * @param startBitIndex The starting index (LSB=0) in the source array for the field.
     * @return A std::array of FieldSize containing the extracted bits.
     *         Returns a zeroed array and prints a warning if extraction is out of bounds.
     */
    template<size_t FieldSize, size_t TotalSize>
    static std::array<unsigned int, FieldSize> ExtractField(const std::array<unsigned int, TotalSize>& source, size_t startBitIndex) {
        std::array<unsigned int, FieldSize> field;
        // Check if the requested slice is within the bounds of the source array.
        if (startBitIndex + FieldSize > TotalSize) {
            std::cerr << "Warning: ExtractField - Slice [" << startBitIndex
                << ".." << startBitIndex + FieldSize - 1 << "] exceeds source bounds [0.."
                << TotalSize - 1 << "]. Returning zeroed array." << std::endl;
            field.fill(0); // Ensure a defined state on error.
            return field;
        }
        // Copy the specified range of bits from source to the new field array.
        std::copy(source.begin() + startBitIndex,          // Start of slice in source
            source.begin() + startBitIndex + FieldSize, // End of slice in source
            field.begin());                             // Start of destination
        return field;
    }

    /**
     * @brief Inserts a smaller source array (field) into a larger destination array.
     * Assumes LSB-first indexing for both source and destination.
     * @tparam FieldSize The number of bits in the source field array.
     * @tparam DestSize The total number of bits in the destination array.
     * @param destination The destination array into which the field will be inserted.
     * @param source The source field array to insert.
     * @param startIndex The starting index (LSB=0) in the destination array where insertion begins.
     * @note Prints an error and does nothing if insertion is out of bounds.
     */
    template<size_t FieldSize, size_t DestSize>
    static void InsertField(std::array<unsigned int, DestSize>& destination,
        const std::array<unsigned int, FieldSize>& source,
        size_t startIndex)
    {
        // Check if the insertion slice fits within the bounds of the destination array.
        if (startIndex + FieldSize > DestSize) {
            std::cerr << "Error: InsertField - Source slice [" << startIndex
                << ".." << startIndex + FieldSize - 1 << "] exceeds destination bounds [0.."
                << DestSize - 1 << "]." << std::endl;
            return; // Abort on error.
        }
        // Copy bits from the source field to the specified position in the destination array.
        std::copy(source.begin(), source.end(), destination.begin() + startIndex);
    }

    // --- Printing Helper Functions ---

    /**
     * @brief Prints the bits of an N-bit array to std::cout, MSB-first for readability.
     * Adds spaces every 4 bits for better visual grouping.
     * @tparam N The number of bits in the array.
     * @param arr The input N-bit array (LSB at arr[0]).
     */
    template<size_t N>
    static void PrintFieldBits(const std::array<unsigned int, N>& arr) {
        if (N == 0) return;
        // Iterate to print from MSB (index N-1) down to LSB (index 0).
        for (size_t i = 0; i < N; ++i) {
            // Calculate the actual array index to print (N-1 is MSB, N-2 is next, etc.)
            size_t index_to_print = (N - 1) - i;
            std::cout << arr[index_to_print];
            // Insert a space after every 4 bits are printed (counting from the MSB side),
            // but not after the very last bit.
            if ((N > 4 || N % 4 == 0) && (i + 1) % 4 == 0 && i < N - 1) {
                std::cout << " ";
            }
        }
    }

    /**
     * @brief Converts an N-bit array (LSB at index 0) to its hexadecimal string representation.
     * @tparam N The number of bits in the array.
     * @param arr The input N-bit array.
     * @param required_hex_digits The number of hex digits to display.
     *                              -1 (default): calculates minimum digits needed for N bits.
     *                              0: returns "0x" if N=0, or "0x0" if N>0 and value is 0.
     *                              Other positive values: pads with leading zeros to this width.
     * @return A string representing the hexadecimal value (e.g., "0x1A").
     */
    template <size_t N>
    static std::string FieldToHexString(const std::array<unsigned int, N>& arr, int required_hex_digits = -1) {
        if (N == 0) {
            return (required_hex_digits == 0) ? "0x" : "0x0"; // Handle N=0 based on desired width
        }

        // Convert the bit array to its unsigned integer value.
        unsigned int val = ArrayToUInt<N>(arr);
        std::stringstream ss;
        ss << "0x"; // Hex prefix.

        // Determine the number of hex digits for formatting.
        int display_digits = (N + 3) / 4; // Minimum hex digits needed for N bits.
        if (required_hex_digits != -1) {
            display_digits = required_hex_digits; // User-specified width overrides.
        }
        // Ensure at least one digit is printed if N > 0 and value is 0, unless width 0 is forced.
        if (display_digits == 0 && val == 0 && N != 0 && required_hex_digits != 0) {
            display_digits = 1;
        }
        // Handle case where N=0 and width 0 specifically.
        if (display_digits == 0 && N == 0 && required_hex_digits == 0) {
            return "0x"; // Just the prefix if width 0 and N=0.
        }


        std::ios_base::fmtflags original_flags = ss.flags(); // Save stream flags.
        // Set hex format, fill with '0', and set width for padding.
        ss << std::hex << std::setfill('0') << std::setw(display_digits) << val;
        ss.flags(original_flags); // Restore original stream flags.

        return ss.str();
    }

    /**
     * @brief Prints an N-bit array to std::cout as a hexadecimal string.
     * @tparam N The number of bits in the array.
     * @param arr The input N-bit array.
     * @param width The number of hex digits to display (passed to FieldToHexString).
     */
    template<size_t N>
    static void PrintFieldHex(const std::array<unsigned int, N>& arr, int width = -1) {
        std::cout << FieldToHexString(arr, width);
    }

    /**
     * @brief Prints an N-bit array to std::cout with a label and its hexadecimal value.
     * Primarily for debugging.
     * @tparam N The number of bits in the array.
     * @param arr The input N-bit array.
     * @param label An optional label to print before the array data.
     */
    template<size_t N>
    static void PrintArray(const std::array<unsigned int, N>& arr, const std::string& label = "") {
        if (!label.empty()) {
            std::cout << label << ": ";
        }
        // Optional: Print binary bits using PrintFieldBits(arr);
        // Print hex value.
        std::cout << " (" << FieldToHexString(arr) << ")" << std::endl;
    }
};

/**
 * @brief Base class for all simulatable hardware components in the processor.
 *
 * This abstract class defines the interface for components that participate
 * in the simulation cycle. Derived classes must implement `DriveUnit` to
 * perform their combinational logic and `WriteOutput` to propagate their
 * results.
 */
class DriveableUnit {
public:
    std::string component_name; ///< Name for the specific instance of the component, for debugging.

    /**
     * @brief Constructor to initialize the component with a name.
     * @param name The name of this component instance. Defaults to "Unnamed Component".
     */
    DriveableUnit(std::string name = "Unnamed Component") : component_name(std::move(name)) {}

    /**
     * @brief Pure virtual function to simulate the combinational logic of the component.
     * Derived classes implement this to read their inputs (usually from `_Src` members),
     * perform calculations, and update their internal state or prepare internal output buffers.
     * @param cycle The current simulation cycle number (optional, for debugging).
     */
    virtual void DriveUnit(int cycle = 0) = 0;

    /**
     * @brief Pure virtual function to simulate writing the component's outputs.
     * Derived classes implement this to take their calculated internal results
     * and write them to their connected output targets (usually `OutputSliceTarget` members).
     * This typically happens after `DriveUnit` has computed the results.
     */
    virtual void WriteOutput() = 0;

    /**
     * @brief Virtual destructor. Default implementation is sufficient.
     */
    virtual ~DriveableUnit() = default;
};

/**
 * @brief Represents an output port of a component that can write a slice of bits
 *        to a destination array.
 * @tparam SourceSize The number of bits this output port produces.
 *
 * This struct manages a pointer to a destination array (or a slice within it)
 * and provides methods to configure this target and write data to it.
 * It assumes LSB-first indexing for all operations.
 */
template <size_t SourceSize>
struct OutputSliceTarget {
    unsigned int* dest_base_ptr = nullptr; ///< Raw pointer to the start of the destination std::array's data.
    size_t dest_start_index = 0;           ///< Starting index (LSB=0) within the destination array where data will be written.
    size_t dest_total_size = 0;            ///< Total size of the destination std::array (for bounds checking).
    std::string port_name = "Unnamed Port"; ///< Name of this specific output port (e.g., "ALU_Result_Out").
    std::string connection_name = "Unconnected"; ///< Name of the connection this port is part of (e.g., "ALU_to_EXMEM_ALUResult").
    std::string parent_component_name = "Unknown Component"; ///< Name of the DriveableUnit that owns this output port.

    /**
     * @brief Constructor to set the port name.
     * @param p_name The name for this output port.
     */
    OutputSliceTarget(std::string p_name = "Unnamed Port") : port_name(std::move(p_name)) {}

    /**
     * @brief Initializes the parent component's name for better error reporting.
     * Called by the owning DriveableUnit during its construction.
     * @param parent Pointer to the owning DriveableUnit.
     */
    void initParent(const DriveableUnit* parent) {
        if (parent) {
            parent_component_name = parent->component_name;
        }
    }

    /**
     * @brief Configures the target destination for this output port to be a specific slice
     *        within a larger destination std::array.
     * @tparam DestSize The size of the target std::array.
     * @param destination_array The std::array to write to.
     * @param start_index The starting bit index (LSB=0) in destination_array for the slice.
     * @param conn_name Optional name for this connection, for debugging.
     */
    template <size_t DestSize>
    void setTarget(std::array<unsigned int, DestSize>& destination_array,
        size_t start_index,
        const std::string& conn_name = "Unconnected") {
        // Bounds check: Ensure the SourceSize bits fit into the destination from start_index.
        if (start_index + SourceSize > DestSize) {
            std::cerr << "Error: OutputSliceTarget::setTarget Component(" << parent_component_name << ") Port(" << port_name << ") Connection(" << conn_name << ") - Slice [" << start_index
                << ".." << start_index + SourceSize - 1 << "] exceeds destination bounds [0.."
                << DestSize - 1 << "]." << std::endl;
            dest_base_ptr = nullptr; dest_start_index = 0; dest_total_size = 0; // Invalidate on error
            return;
        }
        dest_base_ptr = destination_array.data(); // Get raw pointer to array data.
        dest_start_index = start_index;
        dest_total_size = DestSize;
        connection_name = conn_name;
    }

    /**
     * @brief Configures the target destination for this output port to be an *entire*
     *        destination std::array. The size of this port (SourceSize) must match
     *        the size of the destination array (DestSize).
     * @tparam DestSize The size of the target std::array.
     * @param destination_array The std::array to write to.
     * @param conn_name Optional name for this connection, for debugging.
     */
    template <size_t DestSize>
    void setTarget(std::array<unsigned int, DestSize>& destination_array,
        const std::string& conn_name = "Unconnected") {
        // Size check: SourceSize must equal DestSize for whole-array targeting.
        if (SourceSize != DestSize) {
            std::cerr << "Error: OutputSliceTarget::setTarget Component(" << parent_component_name << ") Port(" << port_name << ") Connection(" << conn_name << ") - Size mismatch. "
                << "Source produces size " << SourceSize << ", but destination array has size "
                << DestSize << "." << std::endl;
            dest_base_ptr = nullptr; dest_start_index = 0; dest_total_size = 0; // Invalidate on error
            return;
        }
        dest_base_ptr = destination_array.data();
        dest_start_index = 0; // Writing to the whole array starts at index 0.
        dest_total_size = DestSize;
        connection_name = conn_name;
    }

    /**
     * @brief Checks if the output target has been validly configured.
     * @return True if configured (pointer is not null, size is positive), false otherwise.
     */
    bool isValid() const {
        return dest_base_ptr != nullptr && dest_total_size > 0;
    }

    /**
     * @brief Writes data from a source_data array (of SourceSize) to the configured target.
     * @param source_data The std::array (of SourceSize bits) containing data to write.
     * @return True if write was successful, false if target is invalid or write is out of bounds.
     */
    bool write(const std::array<unsigned int, SourceSize>& source_data) {
        if (!isValid()) {
            std::cerr << "Error: OutputSliceTarget Component(" << parent_component_name
                << ") Port(" << port_name << ") Connection(" << connection_name
                << ") destination not set or invalid." << std::endl;
            return false;
        }
        // Double-check bounds, though setTarget should have caught this.
        if (dest_start_index + SourceSize > dest_total_size) {
            std::cerr << "Error: OutputSliceTarget Component(" << parent_component_name
                << ") Port(" << port_name << ") Connection(" << connection_name
                << ") write out of bounds. "
                << "Start: " << dest_start_index << ", Source Size: " << SourceSize
                << ", Dest Total Size: " << dest_total_size << std::endl;
            return false;
        }
        // Copy 'SourceSize' bits from source_data to the destination array at the specified offset.
        std::copy(source_data.begin(), source_data.end(), dest_base_ptr + dest_start_index);
        return true;
    }
};

/**
 * @brief Represents an input port of a component that can read a slice of bits
 *        from a source array.
 * @tparam DestSize The number of bits this input port expects to read.
 *
 * This struct manages a pointer to a source array (or a slice within it)
 * and provides methods to configure this source and read data from it into a
 * local buffer within the owning component.
 * It assumes LSB-first indexing for all operations.
 */
template <size_t DestSize> // DestSize is the size of the data this input port *consumes*.
struct InputSliceSource {
    const unsigned int* source_base_ptr = nullptr; ///< Raw pointer to the start of the source std::array's data (const as we only read).
    size_t source_start_index = 0;                 ///< Starting index (LSB=0) in the source array from where data will be read.
    size_t source_total_size = 0;                  ///< Total size of the source std::array (for bounds checking).
    std::string port_name = "Unnamed Port";        ///< Name of this specific input port (e.g., "ALU_InputA_Src").
    std::string connection_name = "Unconnected";   ///< Name of the connection this port is part of.
    std::string parent_component_name = "Unknown Component"; ///< Name of the DriveableUnit that owns this input port.

    /**
     * @brief Constructor to set the port name.
     * @param p_name The name for this input port.
     */
    InputSliceSource(std::string p_name = "Unnamed Port") : port_name(std::move(p_name)) {}

    /**
     * @brief Initializes the parent component's name for better error reporting.
     * Called by the owning DriveableUnit during its construction.
     * @param parent Pointer to the owning DriveableUnit.
     */
    void initParent(const DriveableUnit* parent) {
        if (parent) {
            parent_component_name = parent->component_name;
        }
    }

    /**
     * @brief Configures the source for this input port to be a specific slice
     *        from a larger source std::array.
     * @tparam SourceArraySize The size of the source std::array providing the data.
     * @param source_array The std::array to read from.
     * @param start_index The starting bit index (LSB=0) in source_array for the slice.
     * @param conn_name Optional name for this connection, for debugging.
     */
    template <size_t SourceArraySize>
    void setSource(const std::array<unsigned int, SourceArraySize>& source_array,
        size_t start_index,
        const std::string& conn_name = "Unconnected") {
        // Bounds check: Ensure the DestSize bits we *need to read* fit within the source array from start_index.
        if (start_index + DestSize > SourceArraySize) {
            std::cerr << "Error: InputSliceSource::setSource Component(" << parent_component_name << ") Port(" << port_name << ") Connection(" << conn_name << ") - Slice [" << start_index
                << ".." << start_index + DestSize - 1 << "] to be read exceeds source bounds [0.."
                << SourceArraySize - 1 << "]." << std::endl;
            source_base_ptr = nullptr; source_start_index = 0; source_total_size = 0; // Invalidate on error
            return;
        }
        source_base_ptr = source_array.data(); // Get raw pointer to array data.
        source_start_index = start_index;
        source_total_size = SourceArraySize;
        connection_name = conn_name;
    }

    /**
     * @brief Configures the source for this input port to be an *entire* source std::array.
     * The size of this port expects (DestSize) must match the size of the source array (SourceArraySize).
     * @tparam SourceArraySize The size of the source std::array providing the data.
     * @param source_array The std::array to read from.
     * @param conn_name Optional name for this connection, for debugging.
     */
    template <size_t SourceArraySize>
    void setSource(const std::array<unsigned int, SourceArraySize>& source_array,
        const std::string& conn_name = "Unconnected") {
        // Size check: DestSize (what this port consumes) must equal SourceArraySize.
        if (DestSize != SourceArraySize) {
            std::cerr << "Error: InputSliceSource::setSource Component(" << parent_component_name << ") Port(" << port_name << ") Connection(" << conn_name << ") - Size mismatch. "
                << "Destination expects size " << DestSize << ", but source array has size "
                << SourceArraySize << "." << std::endl;
            source_base_ptr = nullptr; source_start_index = 0; source_total_size = 0; // Invalidate on error
            return;
        }
        source_base_ptr = source_array.data();
        source_start_index = 0; // Reading the whole array starts at index 0.
        source_total_size = SourceArraySize;
        connection_name = conn_name;
    }

    /**
     * @brief Checks if the input source has been validly configured.
     * @return True if configured (pointer is not null, size is positive), false otherwise.
     */
    bool isValid() const {
        return source_base_ptr != nullptr && source_total_size > 0;
    }

    /**
     * @brief Reads DestSize bits from the configured source slice into a destination_buffer.
     * @param destination_buffer The std::array (of DestSize bits) where the read data will be stored.
     * @return True if read was successful, false if source is invalid or read is out of bounds.
     */
    bool read(std::array<unsigned int, DestSize>& destination_buffer) {
        if (!isValid()) {
            std::cerr << "Error: InputSliceSource Component(" << parent_component_name
                << ") Port(" << port_name << ") Connection(" << connection_name
                << ") source not set or invalid during read." << std::endl;
            destination_buffer.fill(0); // Zero out buffer on error.
            return false;
        }
        // Double-check bounds: ensure reading DestSize bits from source_start_index
        // does not go beyond the total size of the source array.
        if (source_start_index + DestSize > source_total_size) {
            std::cerr << "Error: InputSliceSource Component(" << parent_component_name
                << ") Port(" << port_name << ") Connection(" << connection_name
                << ") read out of bounds. "
                << "Start: " << source_start_index << ", Read Size: " << DestSize
                << ", Source Total Size: " << source_total_size << std::endl;
            destination_buffer.fill(0); // Zero out buffer on error.
            return false;
        }
        // Copy 'DestSize' bits from the source array (at specified offset) to the destination_buffer.
        std::copy(source_base_ptr + source_start_index,
            source_base_ptr + source_start_index + DestSize,
            destination_buffer.begin());
        return true;
    }
};

/**
 * @brief Simulates the main memory of the processor.
 *
 * Handles instruction fetching based on `InstAddress_Internal` and
 * data memory accesses (read/write) based on `MemAddress_Internal`,
 * `MemRead_Internal`, `MemWrite_Internal`, and `WriteData_Internal`.
 * The memory is modeled as an array of 24-bit words.
 */
class Memory : public DriveableUnit {
public:
    // --- State Variables ---
    /// Main memory storage: 169 words, each 24 bits wide. Bits in each word are LSB-first.
    std::array<std::array<unsigned int, 24>, 169> MemoryFile{};
    /// Internal buffer for the instruction fetch address.
    std::array<unsigned int, 24> InstAddress_Internal;
    /// Internal buffer for the memory write enable signal (1 bit).
    std::array<unsigned int, 1> MemWrite_Internal;
    /// Internal buffer for the memory read enable signal (1 bit).
    std::array<unsigned int, 1> MemRead_Internal;
    /// Internal buffer for the data memory access address.
    std::array<unsigned int, 24> MemAddress_Internal;
    /// Internal buffer for data to be written to memory.
    std::array<unsigned int, 24> WriteData_Internal;
    /// Internal buffer for the instruction fetched from memory.
    std::array<unsigned int, 24> InstDataOut_Internal;
    /// Internal buffer for data read from memory.
    std::array<unsigned int, 24> ReadDataOut_Internal;

    // --- Input Ports ---
    /// Input source for the instruction fetch address.
    InputSliceSource<24> InstAddress_Src{ "InstAddress_Src" };
    /// Input source for the memory write enable signal.
    InputSliceSource<1> MemWrite_Src{ "MemWrite_Src" };
    /// Input source for the memory read enable signal.
    InputSliceSource<1> MemRead_Src{ "MemRead_Src" };
    /// Input source for the data memory access address.
    InputSliceSource<24> MemAddress_Src{ "MemAddress_Src" };
    /// Input source for the data to be written to memory.
    InputSliceSource<24> WriteData_Src{ "WriteData_Src" };

    // --- Output Ports ---
    /// Output target for the fetched instruction.
    OutputSliceTarget<24> INSTRUCTION_OUTPUT{ "INSTRUCTION_OUTPUT" };
    /// Output target for data read from memory.
    OutputSliceTarget<24> READ_DATA_OUTPUT{ "READ_DATA_OUTPUT" };

    /**
     * @brief Constructor for the Memory component.
     * Initializes port names for debugging and clears memory contents and internal buffers.
     * @param name The name of this Memory instance.
     */
    Memory(const std::string& name = "Memory") : DriveableUnit(name) {
        // Initialize all input and output ports with parent component information for debugging.
        InstAddress_Src.initParent(this);
        MemWrite_Src.initParent(this);
        MemRead_Src.initParent(this);
        MemAddress_Src.initParent(this);
        WriteData_Src.initParent(this);
        INSTRUCTION_OUTPUT.initParent(this);
        READ_DATA_OUTPUT.initParent(this);

        // Initialize all memory locations in MemoryFile to 0.
        for (auto& row : MemoryFile) {
            row.fill(0);
        }
        // Initialize all internal state buffers to 0 or their default inactive state.
        InstAddress_Internal.fill(0);
        MemWrite_Internal[0] = 0; // Default: memory write disabled.
        MemRead_Internal[0] = 0;  // Default: memory read disabled.
        MemAddress_Internal.fill(0);
        WriteData_Internal.fill(0);
        InstDataOut_Internal.fill(0); // Default output if fetch is out of bounds.
        ReadDataOut_Internal.fill(0); // Default output if read is out of bounds or not enabled.
    }
    ~Memory() override = default; // Default destructor is sufficient.

    /**
     * @brief Simulates memory operations for one cycle.
     * Reads input signals, performs instruction fetch, and handles data memory reads/writes.
     * @param cycle The current simulation cycle number (for debugging).
     */
    void DriveUnit(int cycle = 0) override {
        // 1. Read all input signals into their respective internal buffers.
        InstAddress_Src.read(InstAddress_Internal);
        MemWrite_Src.read(MemWrite_Internal);
        MemRead_Src.read(MemRead_Internal);
        MemAddress_Src.read(MemAddress_Internal);
        WriteData_Src.read(WriteData_Internal);

        // 2. Perform Instruction Fetch.
        // Convert the 24-bit instruction address array to an unsigned integer.
        unsigned int fetch_address = Convert::ArrayToUInt<24>(InstAddress_Internal);
        if (fetch_address < MemoryFile.size()) { // Check if address is within memory bounds.
            InstDataOut_Internal = MemoryFile[fetch_address]; // Fetch instruction.
        }
        else {
            // Address out of range: output 0s and print a warning.
            std::cerr << "Warning: Instruction fetch address 0x" << std::hex << fetch_address << std::dec << " out of range." << std::endl;
            InstDataOut_Internal.fill(0);
        }

        // 3. Perform Data Memory Access (Read/Write).
        ReadDataOut_Internal.fill(0); // Default data read output to 0.
        // Convert the 24-bit data memory address array to an unsigned integer.
        unsigned int data_address = Convert::ArrayToUInt<24>(MemAddress_Internal);

        // Only proceed if the data address is within memory bounds.
        if (data_address < MemoryFile.size()) {
            // If MemRead signal is asserted (1), read from memory.
            // In MIPS, read happens combinatorially based on address and MemRead.
            if (MemRead_Internal[0] == 1) {
                ReadDataOut_Internal = MemoryFile[data_address];
            }
            // If MemWrite signal is asserted (1), write to memory.
            // In MIPS, write typically happens at the clock edge, but here it's modeled
            // combinationally based on inputs for simplicity in DriveUnit.
            // If both MemRead and MemWrite are asserted for the same address in the same cycle,
            // the read will get the old value, and then the write will update it.
            if (MemWrite_Internal[0] == 1) {
                MemoryFile[data_address] = WriteData_Internal;
            }
        }
        else {
            // If trying to read or write to an out-of-bounds data address, print a warning.
            if (MemRead_Internal[0] == 1 || MemWrite_Internal[0] == 1) {
                std::cerr << "Warning: Data memory address 0x" << std::hex << data_address << std::dec << " out of range." << std::endl;
            }
        }
    }

    /**
     * @brief Writes the results of memory operations (fetched instruction, read data)
     * to the configured output targets.
     */
    void WriteOutput() override {
        // Write the internally buffered instruction data to the INSTRUCTION_OUTPUT port.
        INSTRUCTION_OUTPUT.write(InstDataOut_Internal);
        // Write the internally buffered read data to the READ_DATA_OUTPUT port.
        READ_DATA_OUTPUT.write(ReadDataOut_Internal);
    }

    /**
    * @brief Prints the current state of the memory file for addresses that are multiples of 3.
    * Displays address (hex), value in memory (hex), and value in binary.
    * This modification is to align with a 3-byte word addressing scheme where valid
    * instruction/word addresses are multiples of 3.
    * @param print_all If true, prints all valid word addresses (multiples of 3) in the range.
    *                  If false (default), only prints non-zero memory locations at valid word addresses.
    * @param start_addr The starting address to print from (inclusive). If not a multiple of 3,
    *                   it will be rounded up to the next multiple of 3.
    * @param end_addr The ending address to print to (inclusive). Defaults to the end of memory.
    */
    void PrintMemoryState(bool print_all = false, size_t start_addr = 0, size_t end_addr = static_cast<size_t>(-1), const std::string& indent = "") const {
        // Adjust end_addr if it's the default or out of bounds
        if (end_addr == static_cast<size_t>(-1) || end_addr >= MemoryFile.size()) {
            end_addr = MemoryFile.size() - 1;
        }

        // Adjust start_addr if out of bounds
        if (start_addr >= MemoryFile.size()) {
            std::cout << "Start address 0x" << std::hex << start_addr << " is out of memory bounds." << std::dec << std::endl;
            return;
        }

        // Adjust start_addr to be the next multiple of 3 if it isn't already.
        // This ensures we only consider valid word addresses.
        if (start_addr % 3 != 0) {
            start_addr = ((start_addr / 3) + 1) * 3;
            // Re-check bounds after adjusting start_addr
            if (start_addr >= MemoryFile.size()) {
                std::cout << "Adjusted start address 0x" << std::hex << start_addr << " is out of memory bounds." << std::dec << std::endl;
                return;
            }
        }


        std::cout << indent << "--- Memory File State (Word-Aligned Addresses, Multiples of 3) ---" << std::endl; // Updated title
        std::cout << indent << std::setfill(' ') << std::left // Align left for labels
            << std::setw(10) << "Address"
            << std::setw(15) << "Hex Value"
            << "Binary Value (MSB first)" << std::endl;
        std::cout << indent << "--------------------------------------------------------------------" << std::endl;

        // Iterate through memory, but only process addresses that are multiples of 3.
        for (size_t i = start_addr; i <= end_addr; ++i) {
            // Only process addresses that are multiples of 3
            if (i % 3 != 0) {
                continue; // Skip addresses not divisible by 3
            }

            // Get the 24-bit value from memory
            const auto& mem_word_array = MemoryFile[i];
            unsigned int value_uint = Convert::ArrayToUInt<24>(mem_word_array);

            // Decide whether to print this entry
            bool should_print = print_all;
            if (!print_all && value_uint != 0) { // If not printing all, print if non-zero
                should_print = true;
            }


            if (should_print) {
                // Address in Hex (padded to 4 digits), right-aligned for consistency with typical hex dumps
                std::cout << indent << "0x" << std::hex << std::setfill('0') << std::setw(4) << std::right << i << "    ";
                std::cout << std::left << std::setfill(' ') << std::setw(15); // Prepare for Hex Value

                // Value in Hex (using existing Convert utility)
                // FieldToHexString already includes "0x" prefix.
                // We want 24 bits / 4 bits/hex_digit = 6 hex digits.
                std::cout << Convert::FieldToHexString<24>(mem_word_array, 6);


                // Value in Binary (MSB first, with spaces for readability)
                // Print from MSB (index 23) down to LSB (index 0)
                for (int bit_idx = 23; bit_idx >= 0; --bit_idx) {
                    std::cout << mem_word_array[bit_idx];
                    // Add space every 4 bits (counting from the printed MSB)
                    // The condition (23 - bit_idx) gives 0 for MSB, 1 for next, etc.
                    if ((23 - bit_idx + 1) % 4 == 0 && bit_idx > 0) {
                        std::cout << " ";
                    }
                }
                std::cout << std::dec << std::endl; // Switch back to decimal for next cout operations
            }
        }
        std::cout << indent << "--------------------------------------------------------------------" << std::endl;
    }

};

/**
 * @brief Simulates a 24-bit adder component.
 *
 * Takes two 24-bit inputs (Augend and Addend) and produces a 24-bit sum.
 * It also provides an optional second output for the sum and a 2-bit output
 * for the two most significant bits (MSBs) of the sum, which can be used for
 * jump address calculation (concatenating with PC's MSBs).
 */
class Adder : public DriveableUnit {
public:
    // --- State Variables ---
    /// Internal buffer for the augend (first operand for addition).
    std::array<unsigned int, 24> Augend_Internal;
    /// Internal buffer for the addend (second operand for addition).
    std::array<unsigned int, 24> Addend_Internal;
    /// Internal buffer for the 24-bit sum.
    std::array<unsigned int, 24> ADDER_OUTPUT_Internal;
    /// Internal buffer for the two most significant bits (bit 22 and bit 23) of the sum.
    /// MSB2_OUTPUT_Internal[0] = sum_bit_22, MSB2_OUTPUT_Internal[1] = sum_bit_23.
    std::array<unsigned int, 2> MSB2_OUTPUT_Internal;

    // --- Input Ports ---
    /// Input source for the augend.
    InputSliceSource<24> Augend_Src{ "Augend_Src" };
    /// Input source for the addend.
    InputSliceSource<24> Addend_Src{ "Addend_Src" };

    // --- Output Ports ---
    /// Primary output target for the 24-bit sum.
    OutputSliceTarget<24> ADDER_OUTPUT{ "ADDER_OUTPUT" };
    /// Optional second output target for the 24-bit sum.
    OutputSliceTarget<24> ADDER_OUTPUT_2{ "ADDER_OUTPUT_2 (Optional)" };
    /// Output target for the two MSBs of the sum.
    OutputSliceTarget<2> MSB2_OUTPUT{ "MSB2_OUTPUT" };

    /**
     * @brief Constructor for the Adder component.
     * Initializes port names and internal buffers.
     * @param name The name of this Adder instance.
     */
    Adder(const std::string& name = "Adder") : DriveableUnit(name) {
        // Initialize all input and output ports with parent component information.
        Augend_Src.initParent(this);
        Addend_Src.initParent(this);
        ADDER_OUTPUT.initParent(this);
        ADDER_OUTPUT_2.initParent(this);
        MSB2_OUTPUT.initParent(this);

        // Initialize internal state buffers to 0.
        Augend_Internal.fill(0);
        Addend_Internal.fill(0);
        ADDER_OUTPUT_Internal.fill(0);
        MSB2_OUTPUT_Internal.fill(0);
    }
    ~Adder() override = default; // Default destructor is sufficient.

    /**
     * @brief Simulates the addition operation for one cycle.
     * Reads augend and addend inputs, performs a 24-bit ripple-carry addition,
     * and extracts the two MSBs of the result.
     * @param cycle The current simulation cycle number (for debugging).
     */
    void DriveUnit(int cycle = 0) override {
        // 1. Read input operands into internal buffers.
        Augend_Src.read(Augend_Internal);
        Addend_Src.read(Addend_Internal);

        // 2. Perform 24-bit Ripple Carry Addition (LSB-first).
        // The array elements are LSB-first (index 0 is bit 0).
        unsigned int carry = 0; // Initialize carry-in to the LSB to 0.
        for (size_t i = 0; i < 24; ++i) {
            unsigned int aug_bit = Augend_Internal[i] & 1; // Get bit i of augend.
            unsigned int add_bit = Addend_Internal[i] & 1; // Get bit i of addend.
            unsigned int sum_bits_and_carry = aug_bit + add_bit + carry; // Sum current bits and previous carry.
            ADDER_OUTPUT_Internal[i] = sum_bits_and_carry & 1;      // Current sum bit is LSB of sum_bits_and_carry.
            carry = sum_bits_and_carry >> 1;                        // New carry-out is MSB of sum_bits_and_carry.
        }
        // Any final carry-out beyond bit 23 is discarded (24-bit addition).

        // 3. Extract the Top 2 Bits (bit 22 and bit 23) of the sum.
        // These are stored in LSB-first order in MSB2_OUTPUT_Internal.
        if (24 >= 2) { // Check to ensure array is large enough (always true for N=24).
            MSB2_OUTPUT_Internal[0] = ADDER_OUTPUT_Internal[22]; // Sum's bit 22 goes to LSB of the 2-bit output.
            MSB2_OUTPUT_Internal[1] = ADDER_OUTPUT_Internal[23]; // Sum's bit 23 goes to MSB of the 2-bit output.
        }
        else {
            // Should not happen for a 24-bit adder if we need top 2 bits.
            MSB2_OUTPUT_Internal.fill(0);
        }
    }

    /**
     * @brief Writes the calculated sum and its two MSBs to the configured output targets.
     */
    void WriteOutput() override {
        // Write the 24-bit sum to the primary output port.
        ADDER_OUTPUT.write(ADDER_OUTPUT_Internal);
        // If the optional second sum output is connected, write to it as well.
        if (ADDER_OUTPUT_2.isValid()) {
            ADDER_OUTPUT_2.write(ADDER_OUTPUT_Internal);
        }
        // If the 2-bit MSB output is connected, write the top two bits.
        if (MSB2_OUTPUT.isValid())
        {
            MSB2_OUTPUT.write(MSB2_OUTPUT_Internal);
        }
    }
};

/**
 * @brief Simulates the MIPS Register File.
 *
 * Contains 9 general-purpose 24-bit registers.
 * Supports two combinational read ports (ReadRegister1, ReadRegister2) and
 * one clocked write port (WriteRegister, WriteData, RegWrite).
 * Register $r0 (index 0) always reads as zero and writes to it are ignored.
 * The write operation is split: inputs are latched in DriveUnit from _Next_Input,
 * and the actual write to the Registers array occurs in PerformWrite().
 */
class RegisterFile : public DriveableUnit {
private:
    /**
     * @brief Converts a 4-bit array (LSB at index 0) representing a register number
     *        to an integer array index (0-8).
     * @param regNumArr The 4-bit array for the register number.
     * @return The integer index corresponding to the register number.
     */
    unsigned int RegNumToArrayIndex(const std::array<unsigned int, 4>& regNumArr) {
        return Convert::ArrayToUInt<4>(regNumArr);
    }

public:
    // --- State Variables ---
    /// Storage for 9 general-purpose 24-bit registers.
    /// Registers[i] is register $i. Bit LSB-first.
    std::array<std::array<unsigned int, 24>, 9> Registers{};

    // Internal buffers for read port addresses (latched from _Src inputs).
    std::array<unsigned int, 4> ReadRegister1_Internal;
    std::array<unsigned int, 4> ReadRegister2_Internal;

    // Internal buffers for write port address, data, and control (latched from _Next_Input).
    std::array<unsigned int, 4> WriteRegister_Internal;
    std::array<unsigned int, 24> WriteData_Internal;
    std::array<unsigned int, 1> RegWrite_Internal; // Write enable signal.

    // Internal buffers for data read from read ports.
    std::array<unsigned int, 24> READ_DATA_1_OUTPUT_Internal;
    std::array<unsigned int, 24> READ_DATA_2_OUTPUT_Internal;

    // Buffers for inputs to the write port, to be latched on the next relevant phase.
    // These are populated by WriteRegister_Src, WriteData_Src, RegWrite_Src.
    std::array<unsigned int, 4> ReadRegister1_Next_Input{}; // Not used, reads are direct
    std::array<unsigned int, 4> ReadRegister2_Next_Input{}; // Not used, reads are direct
    std::array<unsigned int, 4> WriteRegister_Next_Input{}; ///< Buffer for next write address.
    std::array<unsigned int, 24> WriteData_Next_Input{};    ///< Buffer for next data to write.
    std::array<unsigned int, 1> RegWrite_Next_Input{};      ///< Buffer for next write enable signal.

    // --- Input Ports ---
    /// Input source for the address of the first register to read (Rs).
    InputSliceSource<4> ReadRegister1_Src{ "ReadRegister1_Src" };
    /// Input source for the address of the second register to read (Rt).
    InputSliceSource<4> ReadRegister2_Src{ "ReadRegister2_Src" };
    /// Input source for the address of the register to write to (Rd or Rt from WB stage).
    InputSliceSource<4> WriteRegister_Src{ "WriteRegister_Src" };
    /// Input source for the data to be written to the register (from WB stage).
    InputSliceSource<24> WriteData_Src{ "WriteData_Src" };
    /// Input source for the register write enable signal (from WB stage).
    InputSliceSource<1> RegWrite_Src{ "RegWrite_Src" };

    // --- Output Ports ---
    /// Output target for data read from the first read port (ReadData1).
    OutputSliceTarget<24> READ_DATA_1_OUTPUT{ "READ_DATA_1_OUTPUT" };
    /// Output target for data read from the second read port (ReadData2).
    OutputSliceTarget<24> READ_DATA_2_OUTPUT{ "READ_DATA_2_OUTPUT" };
    /// Optional output for ReadData1, specifically for the branch comparator.
    OutputSliceTarget<24> BRANCH_COMPARATOR_READ_DATA_1{ "BRANCH_COMPARATOR_READ_DATA_1" };
    /// Optional output for ReadData2, specifically for the branch comparator.
    OutputSliceTarget<24> BRANCH_COMPARATOR_READ_DATA_2{ "BRANCH_COMPARATOR_READ_DATA_2" };

    // Mnemonic mapping for registers (indices 0-8):
    // 0: $r0 (always zero), 1: $v0, 2: $v1, 3: $v2, 4: $v3,
    // 5: $t0, 6: $a0, 7: $a1, 8: $at

    /**
     * @brief Constructor for the RegisterFile component.
     * Initializes ports, registers to zero, and internal buffers.
     * @param name The name of this RegisterFile instance.
     */
    RegisterFile(const std::string& name = "RegisterFile") : DriveableUnit(name) {
        // Initialize all input and output ports with parent component information.
        ReadRegister1_Src.initParent(this);
        ReadRegister2_Src.initParent(this);
        WriteRegister_Src.initParent(this);
        WriteData_Src.initParent(this);
        RegWrite_Src.initParent(this);
        READ_DATA_1_OUTPUT.initParent(this);
        READ_DATA_2_OUTPUT.initParent(this);
        BRANCH_COMPARATOR_READ_DATA_1.initParent(this);
        BRANCH_COMPARATOR_READ_DATA_2.initParent(this);

        // Initialize all physical registers in the file to zero.
        for (auto& reg : Registers) {
            reg.fill(0);
        }
        // Initialize internal buffers for read/write operations and outputs.
        ReadRegister1_Internal.fill(0);
        ReadRegister2_Internal.fill(0);
        WriteRegister_Internal.fill(0);
        WriteData_Internal.fill(0);
        RegWrite_Internal[0] = 0; // Default: register write disabled.
        READ_DATA_1_OUTPUT_Internal.fill(0);
        READ_DATA_2_OUTPUT_Internal.fill(0);
        // Initialize _Next_Input buffers for the write port.
        ReadRegister1_Next_Input.fill(0); // Not strictly used for latching reads
        ReadRegister2_Next_Input.fill(0); // Not strictly used for latching reads
        WriteRegister_Next_Input.fill(0);
        WriteData_Next_Input.fill(0);
        RegWrite_Next_Input.fill(0);
    }
    ~RegisterFile() override = default;

    /**
     * @brief Simulates register file operations for one cycle.
     * In this model, DriveUnit handles two main tasks:
     * 1. Latches the write-side inputs (WriteRegister#, WriteData, RegWrite) from their
     *    _Next_Input buffers (which were populated by _Src inputs during the combinational phase)
     *    into _Internal buffers. This prepares them for PerformWrite().
     * 2. Performs the combinational READ operations based on ReadRegister1_Src and ReadRegister2_Src.
     *    The results are stored in READ_DATA_1_OUTPUT_Internal and READ_DATA_2_OUTPUT_Internal.
     * @param cycle The current simulation cycle number (for debugging).
     */
    void DriveUnit(int cycle = 0) override {
        // --- Latch Write Port Inputs ---
        // Transfer values from _Next_Input (populated by _Src connections from WB stage)
        // into the _Internal buffers that PerformWrite() will use.
        WriteRegister_Internal = WriteRegister_Next_Input;
        WriteData_Internal = WriteData_Next_Input;
        RegWrite_Internal = RegWrite_Next_Input;

        // Clear _Next_Input buffers for the next cycle's WB stage data.
        WriteRegister_Next_Input.fill(0);
        WriteData_Next_Input.fill(0);
        RegWrite_Next_Input.fill(0);

        // --- Populate _Next_Input Buffers for Next Latching Phase ---
        // Read new write-side inputs from sources (typically from MEM/WB pipeline register output).
        // These will be latched into _Internal buffers at the *next* appropriate DriveUnit call for RegisterFile.
        WriteRegister_Src.read(WriteRegister_Next_Input);
        RegWrite_Src.read(RegWrite_Next_Input);
        // WriteData_Src.read(WriteData_Next_Input); // WriteData_Src is connected directly to Registers.WriteData_Next_Input in main

        // --- Perform Combinational Read Operations ---
        // Read addresses for read ports (typically from IF/ID pipeline register).
        ReadRegister1_Src.read(ReadRegister1_Internal);
        ReadRegister2_Src.read(ReadRegister2_Internal);

        // Perform read for port 1.
        unsigned int reg1_idx = RegNumToArrayIndex(ReadRegister1_Internal);
        if (reg1_idx == 0) { // Register $r0 always reads as 0.
            READ_DATA_1_OUTPUT_Internal.fill(0);
        }
        else if (reg1_idx < Registers.size()) { // Valid register index (excluding $r0).
            // Read data from the specified register.
            // This read happens *before* any potential write in the same cycle to the same register.
            // If forwarding is needed for a value being written in the same cycle,
            // that's handled by forwarding units outside the register file.
            READ_DATA_1_OUTPUT_Internal = Registers[reg1_idx];
        }
        else { // Index out of bounds.
            std::cerr << "Warning: Read Register 1 index " << reg1_idx << " out of bounds." << std::endl;
            READ_DATA_1_OUTPUT_Internal.fill(0); // Return 0 on error.
        }

        // Perform read for port 2.
        unsigned int reg2_idx = RegNumToArrayIndex(ReadRegister2_Internal);
        if (reg2_idx == 0) { // Register $r0 always reads as 0.
            READ_DATA_2_OUTPUT_Internal.fill(0);
        }
        else if (reg2_idx < Registers.size()) { // Valid register index.
            READ_DATA_2_OUTPUT_Internal = Registers[reg2_idx];
        }
        else { // Index out of bounds.
            std::cerr << "Warning: Read Register 2 index " << reg2_idx << " out of bounds." << std::endl;
            READ_DATA_2_OUTPUT_Internal.fill(0); // Return 0 on error.
        }

        // Debug print for cycle 8.
        if (cycle == 8)
        {
            std::cout << component_name
                << " ReadRegister1_Internal = " << Convert::ArrayToUInt(ReadRegister1_Internal)
                << " ReadRegister2_Internal = " << Convert::ArrayToUInt(ReadRegister2_Internal)
                << " WriteRegister_Internal = " << Convert::ArrayToUInt(WriteRegister_Internal)
                << " WriteData_Internal = " << Convert::ArrayToUInt(WriteData_Internal)
                << " RegWrite_Internal = " << Convert::ArrayToUInt(RegWrite_Internal)
                << " READ_DATA_1_OUTPUT_Internal = " << Convert::ArrayToUInt(READ_DATA_1_OUTPUT_Internal)
                << " READ_DATA_2_OUTPUT_Internal = " << Convert::ArrayToUInt(READ_DATA_2_OUTPUT_Internal)
                << " ReadRegister1_Next_Input = " << Convert::ArrayToUInt(ReadRegister1_Next_Input) // Should be 0 as it's not used for latching reads
                << " ReadRegister2_Next_Input = " << Convert::ArrayToUInt(ReadRegister2_Next_Input) // Should be 0
                << " WriteRegister_Next_Input = " << Convert::ArrayToUInt(WriteRegister_Next_Input) // Value from WB for *next* write
                << " WriteData_Next_Input = " << Convert::ArrayToUInt(WriteData_Next_Input)       // Value from WB for *next* write
                << " RegWrite_Next_Input = " << Convert::ArrayToUInt(RegWrite_Next_Input)         // Value from WB for *next* write
                << std::endl;
        }
    }

    /**
     * @brief Propagates the data read from the register file (in `DriveUnit`)
     * to the connected output targets. This is part of the combinational path.
     */
    void WriteOutput() override {
        READ_DATA_1_OUTPUT.write(READ_DATA_1_OUTPUT_Internal);
        READ_DATA_2_OUTPUT.write(READ_DATA_2_OUTPUT_Internal);

        // If dedicated branch comparator outputs are connected, write to them as well.
        if (BRANCH_COMPARATOR_READ_DATA_1.isValid()) {
            BRANCH_COMPARATOR_READ_DATA_1.write(READ_DATA_1_OUTPUT_Internal);
        }
        if (BRANCH_COMPARATOR_READ_DATA_2.isValid()) {
            BRANCH_COMPARATOR_READ_DATA_2.write(READ_DATA_2_OUTPUT_Internal);
        }
    }

    /**
     * @brief Performs the actual write operation to the register file.
     * This method should be called at the appropriate point in the clock cycle
     * (typically modeling the write on a clock edge, e.g., end of WB stage).
     * It uses the `WriteRegister_Internal`, `WriteData_Internal`, and `RegWrite_Internal`
     * values that were latched by `DriveUnit`.
     */
    void PerformWrite() {
        // Use the latched internal values for write address, data, and enable.
        unsigned int writeIndex = RegNumToArrayIndex(WriteRegister_Internal);
        unsigned int regWriteSignal = RegWrite_Internal[0]; // Get the 0/1 value.

        // Only write if:
        // 1. RegWrite signal is asserted (1).
        // 2. The write index is not $r0 (index 0).
        // 3. The write index is within the valid bounds of the register file.
        if (regWriteSignal == 1) {
            if (writeIndex > 0 && writeIndex < Registers.size()) {
                // Debug print for the write operation.
                //std::cout << "  Performing write: Registers[" << writeIndex << "] = ";
                //Convert::PrintFieldHex<24>(WriteData_Internal, 6); // Print data in hex.
                //std::cout << std::endl;
                // Perform the actual write to the physical register array.
                Registers[writeIndex] = WriteData_Internal;
            }
            else if (writeIndex == 0) {
                // Writing to $r0 is ignored.
                // std::cerr << "Warning: Attempted write to $r0 ignored." << std::endl;
            }
            else {
                // Write index is out of bounds.
                std::cerr << "Error: Write Register index " << writeIndex << " out of bounds." << std::endl;
            }
        }
    }

    /**
         * @brief Prints the current state of all registers in the register file.
         * Displays register name, number, hexadecimal value, and binary value.
         * Useful for debugging.
         * @param indent An optional string to prepend to each line for indentation.
         */
    void PrintRegisterFileState(const std::string& indent = "") {
        std::cout << indent << "--- Register File State ---" << std::endl;
        // Header for the columns
        std::cout << indent
            << std::left << std::setw(10) << "Reg Name"
            << std::left << std::setw(15) << "Hex Value"
            << "Binary Value (MSB first, grouped by 4)" << std::endl;
        std::cout << indent << "--------------------------------------------------------------------------" << std::endl;


        // Predefined names for registers 0-8 for user-friendly output.
        const std::array<std::string, 9> regNames = {
            "r0", "v0", "v1", "v2", "v3", "t0", "a0", "a1", "at"
        };

        // Iterate through all registers and print their name, number, hex value, and binary value.
        for (size_t i = 0; i < Registers.size(); ++i) {
            // Column 1: Register Name and Number
            std::stringstream regLabel;
            regLabel << "$" << regNames[i] << " (#" << i << ")";
            std::cout << indent << std::left << std::setw(8) << regLabel.str(); // Adjust width as needed

            // Column 2: Hex Value
            std::cout << "  " << std::left << std::setw(15); // Reserve space for hex value
            // Use Convert::FieldToHexString to get the hex string, then print it.
            // This allows better column alignment if FieldToHexString itself prints.
            // Or, more directly if PrintFieldHex is suitable:
            Convert::PrintFieldHex<24>(Registers[i], 6); // Hex value (6 digits for 24 bits)
            std::cout << "   "; // Separator before binary

            // Column 3: Binary Value
            // Print from MSB (index 23) down to LSB (index 0) for the 24-bit register.
            // The Registers[i] array stores bits LSB-first (index 0 is bit 0).
            for (int bit_idx = 23; bit_idx >= 0; --bit_idx) {
                std::cout << Registers[i][bit_idx];
                // Add space after every 4 bits (when looking from MSB), but not after the LSB.
                // (23 - bit_idx) ensures we count from the MSB side for grouping.
                if (bit_idx > 0 && (bit_idx % 4 == 0)) {
                    std::cout << " ";
                }
            }
            std::cout << std::endl;
        }
        std::cout << indent << "--------------------------------------------------------------------------" << std::endl;
    }
};

/**
 * @brief Simulates the Arithmetic Logic Unit (ALU) of the processor.
 *
 * Performs various arithmetic and logical operations based on 4-bit control lines.
 * Takes two 24-bit data inputs (InputA, InputB) and a 4-bit shamt input.
 * Produces a 24-bit result and a 1-bit Zero flag.
 * Operations include ADD, SUB, MULTLOW (signed multiply, lower 24 bits),
 * AND, OR, XOR, SLT (set on less than, signed), SLL (shift left logical),
 * and SRL (shift right logical).
 */
class ArithmeticLogicUnit : public DriveableUnit {
public:
    // --- State Variables ---
    /// Internal buffer for the 4-bit ALU control signals.
    std::array<unsigned int, 4> ALUControlLines_Internal;
    /// Internal buffer for the first 24-bit data input (typically from Rs or a forwarded value).
    std::array<unsigned int, 24> InputA_Internal;
    /// Internal buffer for the second 24-bit data input (typically from Rt, immediate, or a forwarded value).
    std::array<unsigned int, 24> InputB_Internal;
    /// Internal buffer for the 24-bit result of the ALU operation.
    std::array<unsigned int, 24> ALU_RESULT_Internal;
    /// Internal buffer for the Zero flag (1 if ALU_RESULT_Internal is zero, 0 otherwise).
    std::array<unsigned int, 1> ZERO_RESULT_Internal;
    /// Internal buffer for the 4-bit shift amount (shamt), used by SLL and SRL.
    std::array<unsigned int, 4> InputShamt_Internal;

    // --- Input Ports ---
    /// Input source for the 4-bit ALU control signals from ALUController.
    InputSliceSource<4> ALUControlLines_Src{ "ALUControlLines_Src" };
    /// Input source for the first 24-bit data operand.
    InputSliceSource<24> InputA_Src{ "InputA_Src" };
    /// Input source for the second 24-bit data operand or immediate value.
    InputSliceSource<24> InputB_Src{ "InputB_Src" };
    /// Input source for the 4-bit shift amount (shamt field from instruction).
    InputSliceSource<4> InputShamt_Src{ "InputShamt_Src" };

    // --- Output Ports ---
    /// Output target for the 24-bit ALU result.
    OutputSliceTarget<24> ALU_RESULT{ "ALU_RESULT" };
    /// Output target for the Zero flag.
    OutputSliceTarget<1> ZERO_RESULT{ "ZERO_RESULT" };

    // --- ALU Operation Control Codes ---
    // These constants define the numeric values corresponding to specific ALU operations,
    // derived from the 4-bit ALU control lines (interpreted as LSB-first binary).
    const unsigned int ALU_CTRL_VAL_ADD = Convert::ArrayToUInt<4>({ 1, 0, 0, 0 }); // Binary 0001 -> Value 1
    const unsigned int ALU_CTRL_VAL_SUB = Convert::ArrayToUInt<4>({ 0, 1, 0, 0 }); // Binary 0010 -> Value 2
    const unsigned int ALU_CTRL_VAL_MULTLOW = Convert::ArrayToUInt<4>({ 1, 1, 0, 0 }); // Binary 0011 -> Value 3
    const unsigned int ALU_CTRL_VAL_AND = Convert::ArrayToUInt<4>({ 0, 0, 1, 0 }); // Binary 0100 -> Value 4
    const unsigned int ALU_CTRL_VAL_OR = Convert::ArrayToUInt<4>({ 1, 0, 1, 0 }); // Binary 0101 -> Value 5
    const unsigned int ALU_CTRL_VAL_XOR = Convert::ArrayToUInt<4>({ 0, 1, 1, 0 }); // Binary 0110 -> Value 6
    const unsigned int ALU_CTRL_VAL_SLT = Convert::ArrayToUInt<4>({ 1, 1, 1, 0 }); // Binary 0111 -> Value 7
    const unsigned int ALU_CTRL_VAL_SLL = Convert::ArrayToUInt<4>({ 0, 0, 0, 1 }); // Binary 1000 -> Value 8
    const unsigned int ALU_CTRL_VAL_SRL = Convert::ArrayToUInt<4>({ 1, 0, 0, 1 }); // Binary 1001 -> Value 9
    const unsigned int ALU_CTRL_VAL_SPECIAL_IMM_LOGIC = Convert::ArrayToUInt<4>({ 0, 1, 0, 1 }); // Binary 1010 -> Value 10

    /**
     * @brief Constructor for the ArithmeticLogicUnit.
     * Initializes port names and internal buffers.
     * @param name The name of this ALU instance.
     */
    ArithmeticLogicUnit(const std::string& name = "ALU") : DriveableUnit(name) {
        // Initialize all input and output ports with parent component information.
        ALUControlLines_Src.initParent(this);
        InputA_Src.initParent(this);
        InputB_Src.initParent(this);
        ALU_RESULT.initParent(this);
        ZERO_RESULT.initParent(this);
        InputShamt_Src.initParent(this); // Initialize the new shamt input port.

        // Initialize internal state buffers.
        ALUControlLines_Internal.fill(0);
        InputA_Internal.fill(0);
        InputB_Internal.fill(0);
        ALU_RESULT_Internal.fill(0);
        ZERO_RESULT_Internal[0] = 1; // Default Zero flag to 1 (assuming initial result is zero).
        InputShamt_Internal.fill(0);   // Initialize shamt buffer.
    }
    ~ArithmeticLogicUnit() override = default; // Default destructor.

    /**
     * @brief Simulates ALU operations for one cycle.
     * Reads data inputs, shamt, and control lines, then performs the specified operation.
     * Updates ALU_RESULT_Internal and ZERO_RESULT_Internal.
     * @param cycle The current simulation cycle number (for debugging).
     */
    void DriveUnit(int cycle = 0) override {
        // 1. Read all inputs into internal buffers.
        ALUControlLines_Src.read(ALUControlLines_Internal);
        InputA_Src.read(InputA_Internal);
        InputB_Src.read(InputB_Internal);
        InputShamt_Src.read(InputShamt_Internal); // Read the shamt value.

        // 2. Perform ALU Operation based on ALUControlLines_Internal.
        ALU_RESULT_Internal.fill(0); // Default result to 0 for each cycle before operation.
        unsigned int aluCtrlVal = Convert::ArrayToUInt<4>(ALUControlLines_Internal);

        if (aluCtrlVal == ALU_CTRL_VAL_ADD) { // ADD: InputA + InputB
            unsigned int carry = 0;
            for (size_t i = 0; i < 24; ++i) {
                unsigned int a_bit = InputA_Internal[i] & 1;
                unsigned int b_bit = InputB_Internal[i] & 1;
                unsigned int sum = a_bit + b_bit + carry;
                ALU_RESULT_Internal[i] = sum & 1;
                carry = sum >> 1;
            }
        }
        else if (aluCtrlVal == ALU_CTRL_VAL_SUB) { // SUB: InputA - InputB (A + ~B + 1)
            unsigned int carry = 1; // Start with carry-in = 1 for two's complement subtraction.
            for (size_t i = 0; i < 24; ++i) {
                unsigned int a_bit = InputA_Internal[i] & 1;
                unsigned int b_bit_inv = (InputB_Internal[i] & 1) ^ 1; // Invert B's current bit.
                unsigned int sum = a_bit + b_bit_inv + carry;
                ALU_RESULT_Internal[i] = sum & 1;
                carry = sum >> 1;
            }
        }
        else if (aluCtrlVal == ALU_CTRL_VAL_MULTLOW) { // MULTLOW: Signed InputA * InputB, lower 24 bits
            long long valA_signed = Convert::ArrayToInt<24>(InputA_Internal);
            long long valB_signed = Convert::ArrayToInt<24>(InputB_Internal);
            long long result_signed = valA_signed * valB_signed;
            // Store the lower 24 bits of the product.
            ALU_RESULT_Internal = Convert::ToArray(static_cast<unsigned int>(result_signed));
        }
        else if (aluCtrlVal == ALU_CTRL_VAL_AND) { // AND: InputA & InputB
            for (size_t i = 0; i < 24; ++i) {
                ALU_RESULT_Internal[i] = (InputA_Internal[i] & 1) & (InputB_Internal[i] & 1);
            }
        }
        else if (aluCtrlVal == ALU_CTRL_VAL_OR) { // OR: InputA | InputB
            for (size_t i = 0; i < 24; ++i) {
                ALU_RESULT_Internal[i] = (InputA_Internal[i] & 1) | (InputB_Internal[i] & 1);
            }
        }
        else if (aluCtrlVal == ALU_CTRL_VAL_XOR) { // XOR: InputA ^ InputB
            for (size_t i = 0; i < 24; ++i) {
                ALU_RESULT_Internal[i] = (InputA_Internal[i] & 1) ^ (InputB_Internal[i] & 1);
            }
        }
        else if (aluCtrlVal == ALU_CTRL_VAL_SLT) { // SLT: Set on Less Than (signed: InputA < InputB ? 1 : 0)
            int valA_slt = Convert::ArrayToInt<24>(InputA_Internal);
            int valB_slt = Convert::ArrayToInt<24>(InputB_Internal);
            ALU_RESULT_Internal.fill(0); // Default result to 0.
            if (valA_slt < valB_slt) {
                ALU_RESULT_Internal[0] = 1; // Set LSB to 1 if A < B.
            }
        }
        else if (aluCtrlVal == ALU_CTRL_VAL_SLL) { // SLL: Shift Left Logical (InputB << Shamt)
            // Value to shift comes from InputB_Internal (typically Rt for MIPS sll).
            unsigned int valToShift = Convert::ArrayToUInt<24>(InputB_Internal);
            // Shift amount comes from the dedicated InputShamt_Internal.
            unsigned int shiftAmount = Convert::ArrayToUInt<4>(InputShamt_Internal);
            unsigned int result = valToShift << shiftAmount;
            ALU_RESULT_Internal = Convert::ToArray(result);
        }
        else if (aluCtrlVal == ALU_CTRL_VAL_SRL) { // SRL: Shift Right Logical (InputB >> Shamt)
            // Value to shift comes from InputB_Internal (typically Rt for MIPS srl).
            unsigned int valToShift = Convert::ArrayToUInt<24>(InputB_Internal);
            // Shift amount comes from the dedicated InputShamt_Internal.
            unsigned int shiftAmount = Convert::ArrayToUInt<4>(InputShamt_Internal);
            unsigned int result = valToShift >> shiftAmount;
            ALU_RESULT_Internal = Convert::ToArray(result);
            // Debug print specific to SRL operation.
            if (cycle > 8) // Example conditional debug
                std::cout << component_name << " srl: shiftAmount = " << shiftAmount << " valToShift = " << std::hex << "0x" << valToShift << std::dec << " result = " << std::hex << "0x" << result << std::dec << std::endl;
        }
        else if (aluCtrlVal == ALU_CTRL_VAL_SPECIAL_IMM_LOGIC) {
            // InputB_Internal contains the 24-bit sign-extended immediate.
            // We need the original lower 12 bits of this immediate.
            std::array<unsigned int, 12> imm12_arr = Convert::ExtractField<12, 24>(InputB_Internal, 0);
            unsigned int imm12_val = Convert::ArrayToUInt<12>(imm12_arr);

            unsigned int valA_uint = Convert::ArrayToUInt<24>(InputA_Internal);
            unsigned int result_val = 0;

            // Differentiate LUI vs ORI based on InputA (Rs value).
            // For LUI, Rs is $r0, so valA_uint will be 0.
            // For ORI, Rs is used, so valA_uint can be non-zero.
            if (valA_uint == 0) { // Assumed LUI path
                result_val = imm12_val << 12; // LUI: imm << 12
            }
            else { // Assumed ORI path
                // For ORI, the 12-bit immediate should be zero-extended for the OR.
                // Since imm12_val is an unsigned int from a 12-bit field,
                // it's already effectively zero-extended when used in a 24-bit context OR.
                result_val = valA_uint | imm12_val;
            }
            ALU_RESULT_Internal = Convert::ToArray(result_val);
        }
        else {
            // Unknown ALU operation if control lines are not all zero (NOP).
            if (Convert::ArrayToUInt<4>(ALUControlLines_Internal) != 0) {
                std::cerr << "Warning: Unknown ALU Control signal received (Value: " << aluCtrlVal << ") in component " << component_name << std::endl;
            }
            ALU_RESULT_Internal.fill(0); // Default to 0 for unknown/NOP operation.
        }

        // 3. Set Zero flag based on the ALU result.
        ZERO_RESULT_Internal[0] = (Convert::ArrayToUInt<24>(ALU_RESULT_Internal) == 0) ? 1 : 0;

        // Debug print for cycle 8.
        if (cycle == 8)
        {
            std::cout << component_name << " (Arithmetic Logic Unit Component)" << std::endl;
            std::cout << " Input A: "; Convert::PrintArray(InputA_Internal);
            std::cout << " Input B: "; Convert::PrintArray(InputB_Internal);
            std::cout << " Input Shamt: "; Convert::PrintArray(InputShamt_Internal); // Print shamt
            std::cout << " Result: "; Convert::PrintArray(ALU_RESULT_Internal);
            std::cout << " ALU CONTROL LINES: "; Convert::PrintArray(ALUControlLines_Internal);
            std::cout << " Zero Flag: " << ZERO_RESULT_Internal[0] << std::endl;
        }
    }

    /**
     * @brief Writes the ALU result to its output target.
     * The Zero flag output is currently commented out but can be enabled if needed.
     */
    void WriteOutput() override {
        ALU_RESULT.write(ALU_RESULT_Internal);
        // ZERO_RESULT.write(ZERO_RESULT_Internal); // Uncomment to write Zero flag output.
    }
};

/**
 * @brief Decodes ALUOp and Funct fields to generate specific 4-bit control signals for the ALU.
 *
 * This unit implements the logic shown in MIPS control path diagrams where
 * a 2-bit ALUOp (from the main Control Unit) and a 4-bit Funct field (from the
 * instruction, for R-types) determine the precise operation the ALU should perform.
 */
class ALUController : public DriveableUnit {
public:
    // --- State Variables ---
    /// Internal buffer for the 4-bit Funct field (from instruction's bits 0-3 for R-types).
    std::array<unsigned int, 4> Funct_Internal;
    /// Internal buffer for the 2-bit ALUOp signal (from main Control Unit).
    std::array<unsigned int, 2> ALUOp_Internal;
    /// Internal buffer for the generated 4-bit ALU control signal output.
    std::array<unsigned int, 4> ALU_CONTROL_Internal;

    // --- Input Ports ---
    /// Input source for the Funct field.
    InputSliceSource<4> Funct_Src{ "Funct_Src" };
    /// Input source for the ALUOp signal.
    InputSliceSource<2> ALUOp_Src{ "ALUOp_Src" };

    // --- Output Ports ---
    /// Output target for the 4-bit ALU control signal, sent to the ALU.
    OutputSliceTarget<4> ALU_CONTROL_OUTPUT{ "ALU_CONTROL_OUTPUT" };

    // --- ALU Operation Control Codes (matching ALU's definitions) ---
    // These are the target values for ALU_CONTROL_Internal based on decoded inputs.
    const unsigned int TARGET_ALU_CTRL_ADD = Convert::ArrayToUInt<4>({ 1, 0, 0, 0 }); // 0001
    const unsigned int TARGET_ALU_CTRL_SUB = Convert::ArrayToUInt<4>({ 0, 1, 0, 0 }); // 0010
    const unsigned int TARGET_ALU_CTRL_MULTLOW = Convert::ArrayToUInt<4>({ 1, 1, 0, 0 }); // 0011
    const unsigned int TARGET_ALU_CTRL_AND = Convert::ArrayToUInt<4>({ 0, 0, 1, 0 }); // 0100
    const unsigned int TARGET_ALU_CTRL_OR = Convert::ArrayToUInt<4>({ 1, 0, 1, 0 }); // 0101
    const unsigned int TARGET_ALU_CTRL_XOR = Convert::ArrayToUInt<4>({ 0, 1, 1, 0 }); // 0110
    const unsigned int TARGET_ALU_CTRL_SLT = Convert::ArrayToUInt<4>({ 1, 1, 1, 0 }); // 0111
    const unsigned int TARGET_ALU_CTRL_SLL = Convert::ArrayToUInt<4>({ 0, 0, 0, 1 }); // 1000
    const unsigned int TARGET_ALU_CTRL_SRL = Convert::ArrayToUInt<4>({ 1, 0, 0, 1 }); // 1001
    const unsigned int TARGET_ALU_CTRL_SPECIAL_IMM_LOGIC = Convert::ArrayToUInt<4>({ 0, 1, 0, 1 }); // 1010

    /**
     * @brief Constructor for the ALUController.
     * Initializes port names and sets a default ALU control signal (ADD).
     * @param name The name of this ALUController instance.
     */
    ALUController(const std::string& name = "ALUController") : DriveableUnit(name) {
        // Initialize input and output ports with parent component information.
        Funct_Src.initParent(this);
        ALUOp_Src.initParent(this);
        ALU_CONTROL_OUTPUT.initParent(this);

        // Initialize internal state buffers.
        Funct_Internal.fill(0);
        ALUOp_Internal.fill(0);
        // Default ALU_CONTROL_Internal to perform an ADD operation.
        ALU_CONTROL_Internal = Convert::ToArray4Bit(TARGET_ALU_CTRL_ADD);
    }
    ~ALUController() override = default; // Default destructor.

    // MIPS ALUOp and Funct field interpretation guide:
    // ALUOp | Funct   | ALU Action
    // ------|---------|------------------
    // 00    | XXXX  | Add (for lw, sw, addi)
    // 01    | XXXX  | Subtract (for beq, bne)
    // 10    | 0001  | Add (R-type add)
    // 10    | 0010  | Subtract (R-type sub)
    // 11    | XXXX  | lui or ori
    // ...   | ...     | ... (other R-types based on Funct)

    /**
     * @brief Simulates ALUController logic for one cycle.
     * Reads ALUOp and Funct inputs and generates the 4-bit ALU control signal.
     * @param cycle The current simulation cycle number (for debugging).
     */
    void DriveUnit(int cycle = 0) override {
        // 1. Read inputs from sources.
        Funct_Src.read(Funct_Internal); // Funct field from instruction (for R-types)
        ALUOp_Src.read(ALUOp_Internal); // ALUOp from main Control Unit

        // 2. Decode ALUOp and Funct to determine the target 4-bit ALU control value.
        unsigned int aluOpVal = Convert::ArrayToUInt<2>(ALUOp_Internal);
        unsigned int targetCtrlVal = TARGET_ALU_CTRL_ADD; // Default to ADD.

        if (aluOpVal == 0) { // ALUOp "00": Typically for LW, SW, ADDI. ALU performs ADD.
            targetCtrlVal = TARGET_ALU_CTRL_ADD;
        }
        else if (aluOpVal == 1) { // ALUOp "01": Typically for BEQ, BNE. ALU performs SUB for comparison.
            targetCtrlVal = TARGET_ALU_CTRL_SUB;
        }
        else if (aluOpVal == 2) { // ALUOp "10": R-type instruction. Operation determined by Funct field.
            unsigned int functVal = Convert::ArrayToUInt<4>(Funct_Internal);
            switch (functVal) {
            case 1: targetCtrlVal = TARGET_ALU_CTRL_ADD; break;     // Funct 0001 (add)
            case 2: targetCtrlVal = TARGET_ALU_CTRL_SUB; break;     // Funct 0010 (sub)
            case 3: targetCtrlVal = TARGET_ALU_CTRL_MULTLOW; break; // Funct 0011 (multlow)
            case 4: targetCtrlVal = TARGET_ALU_CTRL_AND; break;     // Funct 0100 (and)
            case 5: targetCtrlVal = TARGET_ALU_CTRL_OR; break;      // Funct 0101 (or)
            case 6: targetCtrlVal = TARGET_ALU_CTRL_XOR; break;     // Funct 0110 (xor)
            case 7: targetCtrlVal = TARGET_ALU_CTRL_SLT; break;     // Funct 0111 (slt)
            case 8: targetCtrlVal = TARGET_ALU_CTRL_SLL; break;     // Funct 1000 (sll)
            case 9: targetCtrlVal = TARGET_ALU_CTRL_SRL; break;     // Funct 1001 (srl)
            default:
                // If Funct is non-zero but unrecognized, warn and default to ADD.
                // A Funct of 0 for an R-type (ALUOp=10) might be a NOP or undefined.
                if (functVal != 0) {
                    std::cerr << "Warning: Unknown R-type Funct code: " << functVal << " in component " << component_name << std::endl;
                }
                targetCtrlVal = TARGET_ALU_CTRL_ADD; // Default for safety.
                break;
            }
        }
        else if (aluOpVal == 3) { // NEW: ALUOp "11": for lui, ori
            targetCtrlVal = TARGET_ALU_CTRL_SPECIAL_IMM_LOGIC;
        }

        // 3. Convert the determined numeric control value back to a 4-bit array.
        ALU_CONTROL_Internal = Convert::ToArray4Bit(targetCtrlVal);
    }

    /**
     * @brief Writes the generated 4-bit ALU control signal to its output target.
     */
    void WriteOutput() override {
        ALU_CONTROL_OUTPUT.write(ALU_CONTROL_Internal);
    }
};

/**
 * @brief Manages the four main pipeline registers of the MIPS processor:
 * IF/ID, ID/EX, EX/MEM, and MEM/WB.
 *
 * This class is responsible for latching data from one stage's output (`_Next_Input` buffers)
 * into the corresponding pipeline register on each clock cycle simulated by `DriveUnit`.
 * It also handles control signals for stalling (`Write_Enable_IF_ID`) and flushing
 * (`Flush_IF_ID`) the initial pipeline stages.
 */
class PipelineRegisters : public DriveableUnit {
public:
    // --- State: Current values held by the pipeline registers ---
    // Each array stores bits LSB-first (index 0 is bit 0).
    /// IF/ID Pipeline Register: Holds instruction fetched and PC+increment value.
    std::array<unsigned int, 48> IF_ID{};
    /// ID/EX Pipeline Register: Holds decoded instruction info, register data, control signals for EX.
    std::array<unsigned int, 92> ID_EX{};
    /// EX/MEM Pipeline Register: Holds ALU result, data for memory write, control signals for MEM.
    std::array<unsigned int, 56> EX_MEM{};
    /// MEM/WB Pipeline Register: Holds data from memory or ALU result, control signals for WB.
    std::array<unsigned int, 54> MEM_WB{};

    // --- Input Buffers: Values prepared to be latched into registers on the next clock edge ---
    // These are populated by the output logic of the preceding pipeline stage during the combinational phase.
    /// Data prepared by IF stage for IF/ID register.
    std::array<unsigned int, 48> IF_ID_Next_Input{};
    /// Data prepared by ID stage for ID/EX register.
    std::array<unsigned int, 92> ID_EX_Next_Input{};
    /// Data prepared by EX stage for EX/MEM register.
    std::array<unsigned int, 56> EX_MEM_Next_Input{};
    /// Data prepared by MEM stage for MEM/WB register.
    std::array<unsigned int, 54> MEM_WB_Next_Input{};

    // --- Control Inputs for Pipeline Flow ---
    /// Control signal to enable writing to IF/ID register. 0 = stall, 1 = latch. From Hazard Unit.
    std::array<unsigned int, 1> Write_Enable_IF_ID{ 1 }; // Default: enabled.
    /// Control signal to flush (clear) IF/ID register. 1 = flush. From Control/Branch Logic.
    std::array<unsigned int, 1> Flush_IF_ID{ 0 };        // Default: no flush.

    // --- Bitfield Layout Comments ---
    // These comments describe the intended layout of data within each pipeline register.
    // All indices are LSB=0.
    // Instruction Format (24 bits):
    // R-type: [20-23: Opcode] [16-19: Rs] [12-15: Rt] [8-11: Rd] [4-7: Shamt] [0-3: Funct]
    // I-type: [20-23: Opcode] [16-19: Rs] [12-15: Rt] [0-11: Constant or Address]
    // J-type: [20-23: Opcode] [0-19: Target Address]
    //
    // Pipeline Register Contents (Indices are LSB=0):
    // IF_ID (48 bits):  [0-23: Instruction] [24-47: PC+increment_value Address]
    // ID_EX (92 bits):  [0-3: Rd#] [4-7: Rt#] [8-11: Rs#] [12-35: Imm Sign Ext]
    //                   [36-59: Register 2 Data] [60-83: Register 1 Data]
    //                   [84: ALUSrc] [85-86: ALUOp] [87: RegDst]
    //                   [88-89: M Ctrls (MemWrite, MemRead)] [90-91: WB Ctrls (MemToReg, RegWrite)]
    // EX_MEM (56 bits): [0-3: Write Register#] [4-27: Write Data (for SW, from Rt)]
    //                   [28-51: ALU Result/Memory Address]
    //                   [52: MemWrite] [53: MemRead]
    //                   [54-55: WB Ctrls (MemToReg, RegWrite)]
    // MEM_WB (54 bits): [0-3: Write Register#] [4-27: ALU Result (from EX/MEM)]
    //                   [28-51: Read Data (from Memory)]
    //                   [52: MemToReg] [53: RegWrite]

    /**
     * @brief Constructor for the PipelineRegisters component.
     * Initializes all pipeline register arrays and their input buffers to zero.
     * Sets default values for IF/ID control signals (enabled, no flush).
     * @param name The name of this PipelineRegisters instance.
     */
    PipelineRegisters(const std::string& name = "PipelineRegisters") : DriveableUnit(name) {
        // Initialize all state arrays (pipeline registers) to all zeros.
        IF_ID.fill(0); ID_EX.fill(0); EX_MEM.fill(0); MEM_WB.fill(0);
        // Initialize all input buffer arrays to all zeros.
        IF_ID_Next_Input.fill(0); ID_EX_Next_Input.fill(0);
        EX_MEM_Next_Input.fill(0); MEM_WB_Next_Input.fill(0);
        // Set default control signal values.
        Write_Enable_IF_ID[0] = 1; // IF/ID latching is enabled by default.
        Flush_IF_ID[0] = 0;        // No flush by default.
    }
    ~PipelineRegisters() override = default; // Default destructor.

    /**
     * @brief Simulates the latching action of the pipeline registers at a clock edge.
     * Data from `_Next_Input` buffers is transferred to the main register arrays.
     * Handles stalling and flushing for the IF/ID register.
     * @param cycle The current simulation cycle number (for debugging).
     */
    void DriveUnit(int cycle = 0) override {
        // --- Latch MEM/WB Register ---
        // Data from the MEM stage's output buffer becomes the new state of MEM/WB.
        MEM_WB = MEM_WB_Next_Input;
        MEM_WB_Next_Input.fill(0); // Clear the input buffer for the next cycle.

        // --- Latch EX/MEM Register ---
        EX_MEM = EX_MEM_Next_Input;
        EX_MEM_Next_Input.fill(0); // Clear buffer.

        // --- Latch ID/EX Register ---
        ID_EX = ID_EX_Next_Input;
        ID_EX_Next_Input.fill(0); // Clear buffer.

        // --- Latch IF/ID Register (Conditional based on Stall and Flush controls) ---
        if (Write_Enable_IF_ID[0] == 1) { // If IF/ID writing is enabled (pipeline not stalled here)
            if (Flush_IF_ID[0] == 1) {   // If a flush signal is active (e.g., taken branch/jump)
                // Flush: Load IF/ID with all zeros (effectively a NOP).
                IF_ID.fill(0); // Using .fill(0) directly on std::array.
            }
            else {
                // No flush and not stalled: Latch the prepared input from IF stage.
                IF_ID = IF_ID_Next_Input;
            }
        }
        // else: If Write_Enable_IF_ID[0] == 0 (stalled), IF_ID retains its previous value.
        // The IF_ID_Next_Input is effectively discarded for this cycle for the IF_ID register.

        // Debug print for cycle 8.
        if (cycle == 8)
        {
            std::cout << component_name << " Flush_IF_ID = " << Flush_IF_ID[0] << " Write_Enable_IF_ID = " << Write_Enable_IF_ID[0] << std::endl;
        }

        // Clear the IF_ID input buffer regardless of whether it was latched or not,
        // to prepare for the IF stage output of the *next* cycle.
        IF_ID_Next_Input.fill(0);

        // --- Reset single-cycle control signals for the IF/ID register ---
        // These signals (Write_Enable_IF_ID, Flush_IF_ID) are typically asserted by
        // other units (HazardDetector, ControlUnit/BranchLogic) for one cycle.
        // Resetting them here ensures they default to a non-asserted state for the
        // start of the next cycle's combinational logic, unless driven again.
        Write_Enable_IF_ID[0] = 1; // Default to write enabled for the next cycle.
        Flush_IF_ID[0] = 0;        // Default to no flush for the next cycle.
    }

    /**
     * @brief Pipeline registers do not have direct outputs in the same way as
     *        combinational units. Their state is read directly by subsequent pipeline stages.
     *        This function is a no-op.
     */
    void WriteOutput() override {
        // No explicit output writing needed. Components in the next stage
        // will read directly from the public state arrays (IF_ID, ID_EX, etc.)
        // or have their InputSliceSource configured to point to them.
    }
};

/**
 * @brief Simulates a generic 2-input, 1-output, 24-bit wide multiplexer.
 *
 * Selects one of two 24-bit data inputs (InputA or InputB) based on a
 * single-bit selector signal. The selected input is passed to the output.
 * Optional secondary and tertiary outputs can also be connected.
 */
class Multiplexor2Way24 : public DriveableUnit {
public:
    // --- State Variables ---
    /// Internal buffer for the 1-bit selector signal.
    std::array<unsigned int, 1> Selector_Internal;
    /// Internal buffer for the first 24-bit data input (selected if Selector is 0).
    std::array<unsigned int, 24> InputA_Internal;
    /// Internal buffer for the second 24-bit data input (selected if Selector is 1).
    std::array<unsigned int, 24> InputB_Internal;
    /// Internal buffer for the 24-bit output.
    std::array<unsigned int, 24> OUTPUT_Internal;

    // --- Input Ports ---
    /// Input source for the 1-bit selector signal.
    InputSliceSource<1> Selector_Src{ "Selector_Src" };
    /// Input source for the first 24-bit data input (Input 0).
    InputSliceSource<24> InputA_Src{ "InputA_Src" };
    /// Input source for the second 24-bit data input (Input 1).
    InputSliceSource<24> InputB_Src{ "InputB_Src" };

    // --- Output Ports ---
    /// Primary output target for the selected 24-bit data.
    OutputSliceTarget<24> OUTPUT{ "OUTPUT" };
    /// Optional second output target.
    OutputSliceTarget<24> OUTPUT_2{ "OUTPUT_2 (Optional)" };
    /// Optional third output target.
    OutputSliceTarget<24> OUTPUT_3{ "OUTPUT_3 (Optional)" };

    /**
     * @brief Constructor for the Multiplexor2Way24 component.
     * Initializes port names and internal buffers.
     * @param name The name of this Multiplexor2Way24 instance.
     */
    Multiplexor2Way24(const std::string& name = "Mux2Way24") : DriveableUnit(name) {
        // Initialize all input and output ports with parent component information.
        Selector_Src.initParent(this);
        InputA_Src.initParent(this);
        InputB_Src.initParent(this);
        OUTPUT.initParent(this);
        OUTPUT_2.initParent(this);
        OUTPUT_3.initParent(this);

        // Initialize internal state buffers.
        Selector_Internal[0] = 0; // Default selector to 0.
        InputA_Internal.fill(0);
        InputB_Internal.fill(0);
        OUTPUT_Internal.fill(0);   // Default output to 0 (corresponds to InputA if selector=0).
    }
    ~Multiplexor2Way24() override = default; // Default destructor.

    /**
     * @brief Simulates the multiplexer logic for one cycle.
     * Reads the selector and data inputs, then selects InputA or InputB
     * to be passed to OUTPUT_Internal.
     * @param cycle The current simulation cycle number (for debugging).
     */
    void DriveUnit(int cycle = 0) override {
        // 1. Read all inputs into their respective internal buffers.
        Selector_Src.read(Selector_Internal);
        InputA_Src.read(InputA_Internal);
        InputB_Src.read(InputB_Internal);

        // 2. Perform selection based on the (single) selector bit.
        if (Selector_Internal[0] == 0) {
            OUTPUT_Internal = InputA_Internal; // Select Input A.
        }
        else { // Selector_Internal[0] == 1
            OUTPUT_Internal = InputB_Internal; // Select Input B.
        }

        // Debug print for cycle 8, specific to JumpMux or BranchAddressMux instances.
        if (cycle == 8)
        {
            if (component_name == "JumpMux" || component_name == "BranchAddressMux") {
                std::cout << component_name << " selector value: " << Convert::ArrayToUInt(Selector_Internal)
                    << " output value: " << Convert::ArrayToInt(OUTPUT_Internal) // Assuming output can be signed for PC-related values
                    << " InputA: " << Convert::ArrayToUInt(InputA_Internal)
                    << " InputB: " << Convert::ArrayToUInt(InputB_Internal) << std::endl;
            }
        }
    }

    /**
     * @brief Writes the selected 24-bit data from OUTPUT_Internal to the
     * configured output target(s).
     */
    void WriteOutput() override {
        OUTPUT.write(OUTPUT_Internal); // Write to primary output.
        // If optional outputs are connected (valid), write to them as well.
        if (OUTPUT_2.isValid()) OUTPUT_2.write(OUTPUT_Internal);
        if (OUTPUT_3.isValid()) OUTPUT_3.write(OUTPUT_Internal);
    }
};

/**
 * @brief Simulates a generic 3-input, 1-output, 24-bit wide multiplexer.
 *
 * Selects one of three 24-bit data inputs (InputA, InputB, or InputC) based on a
 * 2-bit selector signal. The selected input is passed to the output.
 * Selector values: 00 -> InputA, 01 -> InputB, 10 -> InputC.
 * An optional secondary output can also be connected.
 */
class Multiplexor3Way24 : public DriveableUnit {
public:
    // --- State Variables ---
    /// Internal buffer for the 2-bit selector signal (LSB at index 0).
    std::array<unsigned int, 2> Selector3Way_Internal;
    /// Internal buffer for the first 24-bit data input (selected if Selector is 00).
    std::array<unsigned int, 24> InputA_Internal;
    /// Internal buffer for the second 24-bit data input (selected if Selector is 01).
    std::array<unsigned int, 24> InputB_Internal;
    /// Internal buffer for the third 24-bit data input (selected if Selector is 10).
    std::array<unsigned int, 24> InputC_Internal;
    /// Internal buffer for the 24-bit output.
    std::array<unsigned int, 24> OUTPUT_Internal;

    // --- Input Ports ---
    /// Input source for the 2-bit selector signal.
    InputSliceSource<2> Selector3Way_Src{ "Selector3Way_Src" };
    /// Input source for data input A (selected by "00").
    InputSliceSource<24> InputA_Src{ "InputA_Src" };
    /// Input source for data input B (selected by "01").
    InputSliceSource<24> InputB_Src{ "InputB_Src" };
    /// Input source for data input C (selected by "10").
    InputSliceSource<24> InputC_Src{ "InputC_Src" };

    // --- Output Ports ---
    /// Primary output target for the selected 24-bit data.
    OutputSliceTarget<24> OUTPUT{ "OUTPUT" };
    /// Optional second output target.
    OutputSliceTarget<24> OUTPUT_2{ "OUTPUT_2 (Optional)" };

    /**
     * @brief Constructor for the Multiplexor3Way24 component.
     * Initializes port names and internal buffers.
     * @param name The name of this Multiplexor3Way24 instance.
     */
    Multiplexor3Way24(const std::string& name = "Mux3Way24") : DriveableUnit(name) {
        // Initialize all input and output ports with parent component information.
        Selector3Way_Src.initParent(this);
        InputA_Src.initParent(this);
        InputB_Src.initParent(this);
        InputC_Src.initParent(this);
        OUTPUT.initParent(this);
        OUTPUT_2.initParent(this);

        // Initialize internal state buffers.
        Selector3Way_Internal.fill(0); // Default selector to 00.
        InputA_Internal.fill(0);
        InputB_Internal.fill(0);
        InputC_Internal.fill(0);
        OUTPUT_Internal.fill(0);      // Default output to 0 (corresponds to InputA if selector=00).
    }
    ~Multiplexor3Way24() override = default; // Default destructor.

    /**
     * @brief Simulates the multiplexer logic for one cycle.
     * Reads the selector and data inputs, then selects InputA, InputB, or InputC
     * to be passed to OUTPUT_Internal.
     * @param cycle The current simulation cycle number (for debugging).
     */
    void DriveUnit(int cycle = 0) override {
        // 1. Read all inputs into their respective internal buffers.
        Selector3Way_Src.read(Selector3Way_Internal);
        InputA_Src.read(InputA_Internal);
        InputB_Src.read(InputB_Internal);
        InputC_Src.read(InputC_Internal);

        // 2. Perform selection based on the 2-bit selector value.
        // Convert the LSB-first selector array to an unsigned integer.
        unsigned int selectorValue = Convert::ArrayToUInt<2>(Selector3Way_Internal);

        switch (selectorValue) {
        case 0: // Selector "00"
            OUTPUT_Internal = InputA_Internal;
            break;
        case 1: // Selector "01"
            OUTPUT_Internal = InputB_Internal;
            break;
        case 2: // Selector "10"
            OUTPUT_Internal = InputC_Internal;
            break;
        default: // Selector "11" or any other invalid bit pattern for a 2-bit selector.
            // This case should ideally not be reached if inputs are constrained.
            std::cerr << "Warning: Invalid 3-Way Mux selector value: " << selectorValue
                << " in component " << component_name << ". Defaulting to Input A." << std::endl;
            OUTPUT_Internal = InputA_Internal; // Default to Input A on error.
            break;
        }

        // Debug print for cycle 8.
        if (cycle == 8)
        {
            std::cout << component_name << " Selector3Way_Internal = " << Convert::ArrayToUInt(Selector3Way_Internal)
                << " OUTPUT_Internal = " << Convert::ArrayToUInt(OUTPUT_Internal) << std::endl;
        }
    }

    /**
     * @brief Writes the selected 24-bit data from OUTPUT_Internal to the
     * configured output target(s).
     */
    void WriteOutput() override {
        OUTPUT.write(OUTPUT_Internal); // Write to primary output.
        // If the optional second output is connected, write to it as well.
        if (OUTPUT_2.isValid()) {
            OUTPUT_2.write(OUTPUT_Internal);
        }
    }
};

/**
 * @brief Simulates a generic 4-input, 1-output, 24-bit wide multiplexer.
 *
 * Selects one of four 24-bit data inputs (InputA, InputB, InputC, or InputD)
 * based on a 2-bit selector signal. The selected input is passed to the output.
 * Selector values: 00->InputA, 01->InputB, 10->InputC, 11->InputD.
 */
class Multiplexor4Way24 : public DriveableUnit {
public:
    // --- State Variables ---
    /// Internal buffer for the 2-bit selector signal (LSB at index 0).
    std::array<unsigned int, 2> Selector_Internal;
    /// Internal buffer for data input A (selected if Selector is 00).
    std::array<unsigned int, 24> InputA_Internal;
    /// Internal buffer for data input B (selected if Selector is 01).
    std::array<unsigned int, 24> InputB_Internal;
    /// Internal buffer for data input C (selected if Selector is 10).
    std::array<unsigned int, 24> InputC_Internal;
    /// Internal buffer for data input D (selected if Selector is 11).
    std::array<unsigned int, 24> InputD_Internal;
    /// Internal buffer for the 24-bit output.
    std::array<unsigned int, 24> OUTPUT_Internal;

    // --- Input Ports ---
    /// Input source for the 2-bit selector signal.
    InputSliceSource<2> Selector_Src{ "Selector_Src" };
    /// Input source for data input A (Path 00).
    InputSliceSource<24> InputA_Src{ "InputA_Src" };
    /// Input source for data input B (Path 01).
    InputSliceSource<24> InputB_Src{ "InputB_Src" };
    /// Input source for data input C (Path 10).
    InputSliceSource<24> InputC_Src{ "InputC_Src" };
    /// Input source for data input D (Path 11).
    InputSliceSource<24> InputD_Src{ "InputD_Src" };

    // --- Output Ports ---
    /// Output target for the selected 24-bit data.
    OutputSliceTarget<24> OUTPUT{ "OUTPUT" };

    /**
     * @brief Constructor for the Multiplexor4Way24 component.
     * Initializes port names and internal buffers.
     * @param name The name of this Multiplexor4Way24 instance.
     */
    Multiplexor4Way24(const std::string& name = "Mux4Way24") : DriveableUnit(name) {
        // Initialize all input and output ports with parent component information.
        Selector_Src.initParent(this);
        InputA_Src.initParent(this);
        InputB_Src.initParent(this);
        InputC_Src.initParent(this);
        InputD_Src.initParent(this);
        OUTPUT.initParent(this);

        // Initialize internal state buffers.
        Selector_Internal.fill(0); // Default selector to 00.
        InputA_Internal.fill(0);
        InputB_Internal.fill(0);
        InputC_Internal.fill(0);
        InputD_Internal.fill(0);
        OUTPUT_Internal.fill(0);   // Default output to 0 (corresponds to InputA if selector=00).
    }
    ~Multiplexor4Way24() override = default; // Default destructor.

    /**
     * @brief Simulates the multiplexer logic for one cycle.
     * Reads the selector and data inputs, then selects InputA, B, C, or D
     * to be passed to OUTPUT_Internal.
     * @param cycle The current simulation cycle number (for debugging).
     */
    void DriveUnit(int cycle = 0) override {
        // 1. Read all inputs into their respective internal buffers.
        Selector_Src.read(Selector_Internal);
        InputA_Src.read(InputA_Internal);
        InputB_Src.read(InputB_Internal);
        InputC_Src.read(InputC_Internal);
        InputD_Src.read(InputD_Internal);

        // 2. Perform selection based on the 2-bit selector value.
        unsigned int selectorValue = Convert::ArrayToUInt<2>(Selector_Internal);

        switch (selectorValue) {
        case 0: OUTPUT_Internal = InputA_Internal; break; // Selector "00"
        case 1: OUTPUT_Internal = InputB_Internal; break; // Selector "01"
        case 2: OUTPUT_Internal = InputC_Internal; break; // Selector "10"
        case 3: OUTPUT_Internal = InputD_Internal; break; // Selector "11"
        default: // This case should not be reached with a 2-bit selector if inputs are valid.
            std::cerr << "Warning: Invalid 4-Way Mux selector value: " << selectorValue
                << " in component " << component_name << ". Defaulting to Input A." << std::endl;
            OUTPUT_Internal = InputA_Internal; // Default to Input A on error.
            break;
        }
        // Debug print for cycle 8, specific to ID stage forwarding muxes.
        if (cycle == 8)
        {
            if (component_name == "ID_ForwardAMux4Way" || component_name == "ID_ForwardBMux4Way") {
                std::cout << " " << component_name << " Selector = " << selectorValue
                    << " (Output: " << Convert::FieldToHexString(OUTPUT_Internal)
                    << ", A=" << Convert::FieldToHexString(InputA_Internal)
                    << ", B=" << Convert::FieldToHexString(InputB_Internal)
                    << ", C=" << Convert::FieldToHexString(InputC_Internal)
                    << ", D=" << Convert::FieldToHexString(InputD_Internal)
                    << ")" << std::endl;
            }
        }
    }

    /**
     * @brief Writes the selected 24-bit data from OUTPUT_Internal to the
     * configured output target.
     */
    void WriteOutput() override {
        OUTPUT.write(OUTPUT_Internal);
    }
};

/**
 * @brief Simulates a generic 2-input, 1-output, 4-bit wide multiplexer.
 *
 * Selects one of two 4-bit data inputs (InputA or InputB) based on a
 * single-bit selector signal. The selected input is passed to the output.
 * Typically used for selecting register numbers (e.g., RegDst Mux).
 */
class Multiplexor2Way4 : public DriveableUnit {
public:
    // --- State Variables ---
    /// Internal buffer for the 1-bit selector signal.
    std::array<unsigned int, 1> Selector_Internal;
    /// Internal buffer for the first 4-bit data input (selected if Selector is 0).
    std::array<unsigned int, 4> InputA_Internal;
    /// Internal buffer for the second 4-bit data input (selected if Selector is 1).
    std::array<unsigned int, 4> InputB_Internal;
    /// Internal buffer for the 4-bit output.
    std::array<unsigned int, 4> OUTPUT_Internal;

    // --- Input Ports ---
    /// Input source for the 1-bit selector signal.
    InputSliceSource<1> Selector_Src{ "Selector_Src" };
    /// Input source for data input A.
    InputSliceSource<4> InputA_Src{ "InputA_Src" };
    /// Input source for data input B.
    InputSliceSource<4> InputB_Src{ "InputB_Src" };

    // --- Output Ports ---
    /// Output target for the selected 4-bit data.
    OutputSliceTarget<4> OUTPUT{ "OUTPUT" };

    /**
     * @brief Constructor for the Multiplexor2Way4 component.
     * Initializes port names and internal buffers.
     * @param name The name of this Multiplexor2Way4 instance.
     */
    Multiplexor2Way4(const std::string& name = "Mux2Way4") : DriveableUnit(name) {
        // Initialize all input and output ports with parent component information.
        Selector_Src.initParent(this);
        InputA_Src.initParent(this);
        InputB_Src.initParent(this);
        OUTPUT.initParent(this);

        // Initialize internal state buffers.
        Selector_Internal[0] = 0; // Default selector to 0.
        InputA_Internal.fill(0);
        InputB_Internal.fill(0);
        OUTPUT_Internal.fill(0);   // Default output to 0.
    }
    ~Multiplexor2Way4() override = default; // Default destructor.

    /**
     * @brief Simulates the multiplexer logic for one cycle.
     * Reads inputs and selects InputA or InputB for OUTPUT_Internal.
     * @param cycle The current simulation cycle number (for debugging).
     */
    void DriveUnit(int cycle = 0) override {
        // 1. Read inputs.
        Selector_Src.read(Selector_Internal);
        InputA_Src.read(InputA_Internal);
        InputB_Src.read(InputB_Internal);

        // 2. Perform selection.
        if (Selector_Internal[0] == 0) {
            OUTPUT_Internal = InputA_Internal;
        }
        else { // Selector_Internal[0] == 1
            OUTPUT_Internal = InputB_Internal;
        }

        // Debug print for cycle 8.
        if (cycle == 8)
        {
            std::cout << " " << component_name << " Selector_Internal = " << Convert::ArrayToUInt(Selector_Internal)
                << " OUTPUT_Internal = " << Convert::ArrayToUInt(OUTPUT_Internal)
                << " InputA_Internal = " << Convert::ArrayToUInt(InputA_Internal)
                << " InputB_Internal = " << Convert::ArrayToUInt(InputB_Internal) << std::endl;
        }
    }

    /**
     * @brief Writes the selected 4-bit data to the output target.
     */
    void WriteOutput() override {
        OUTPUT.write(OUTPUT_Internal);
    }
};

/**
 * @brief Simulates a generic 2-input, 1-output, 8-bit wide multiplexer.
 *
 * Selects one of two 8-bit data inputs (InputA or InputB) based on a
 * single-bit selector signal.
 * Often used to select between normal control signals and NOP (zeroed)
 * control signals, for example, when stalling or flushing.
 */
class Multiplexor2Way8 : public DriveableUnit {
public:
    // --- State Variables ---
    /// Internal buffer for the 1-bit selector signal.
    std::array<unsigned int, 1> Selector_Internal;
    /// Internal buffer for the first 8-bit data input (e.g., normal control lines).
    std::array<unsigned int, 8> InputA_Internal;
    /// Internal buffer for the second 8-bit data input (e.g., NOP control lines, all zeros).
    std::array<unsigned int, 8> InputB_Internal;
    /// Internal buffer for the 8-bit output.
    std::array<unsigned int, 8> OUTPUT_Internal;

    // --- Input Ports ---
    /// Input source for the 1-bit selector (e.g., from Hazard Unit to select NOP controls).
    InputSliceSource<1> Selector_Src{ "Selector_Src" };
    /// Input source for data input A (e.g., from main Control Unit).
    InputSliceSource<8> InputA_Src{ "InputA_Src" };
    /// Input source for data input B (e.g., hardwired NOP value or another source).
    InputSliceSource<8> InputB_Src{ "InputB_Src" };

    // --- Output Ports ---
    /// Output target for the selected 8-bit data (e.g., to ID/EX register's control bit section).
    OutputSliceTarget<8> OUTPUT{ "OUTPUT" };

    /**
     * @brief Constructor for the Multiplexor2Way8 component.
     * Initializes port names and internal buffers.
     * @param name The name of this Multiplexor2Way8 instance.
     */
    Multiplexor2Way8(const std::string& name = "Mux2Way8") : DriveableUnit(name) {
        // Initialize all input and output ports with parent component information.
        Selector_Src.initParent(this);
        InputA_Src.initParent(this);
        InputB_Src.initParent(this);
        OUTPUT.initParent(this);

        // Initialize internal state buffers.
        Selector_Internal[0] = 0; // Default selector to 0.
        InputA_Internal.fill(0);
        InputB_Internal.fill(0);   // Input B might often be all zeros for NOP.
        OUTPUT_Internal.fill(0);   // Default output to 0.
    }
    ~Multiplexor2Way8() override = default; // Default destructor.

    /**
     * @brief Simulates the multiplexer logic for one cycle.
     * Reads inputs and selects InputA or InputB for OUTPUT_Internal.
     * @param cycle The current simulation cycle number (for debugging).
     */
    void DriveUnit(int cycle = 0) override {
        // 1. Read inputs.
        Selector_Src.read(Selector_Internal);
        InputA_Src.read(InputA_Internal);
        InputB_Src.read(InputB_Internal);

        // 2. Perform selection.
        if (Selector_Internal[0] == 0) {
            OUTPUT_Internal = InputA_Internal;
        }
        else { // Selector_Internal[0] == 1
            OUTPUT_Internal = InputB_Internal;
        }

        // Debug print for cycle 8.
        if (cycle == 8)
        {
            std::cout << component_name << " Selector_Internal = " << Convert::ArrayToUInt(Selector_Internal)
                << " OUTPUT_Internal = " << Convert::ArrayToUInt(OUTPUT_Internal) << std::endl;
        }
    }

    /**
     * @brief Writes the selected 8-bit data to the output target.
     */
    void WriteOutput() override {
        OUTPUT.write(OUTPUT_Internal);
    }
};

/**
 * @brief Simulates the Program Counter (PC) register.
 *
 * Holds the address of the next instruction to be fetched.
 * Can be updated with a new value (`NextValueIn_Internal`) if `WriteEnable_Internal` is asserted.
 * Outputs its current value to two targets, typically for instruction fetching
 * and for calculating PC + increment_value.
 */
class ProgramCounter : public DriveableUnit {
public:
    // --- State Variables ---
    /// Current 24-bit value of the Program Counter (LSB at index 0).
    std::array<unsigned int, 24> Value;
    /// Internal buffer for the next PC value to be latched.
    std::array<unsigned int, 24> NextValueIn_Internal;
    /// Internal buffer for the write enable signal (1-bit). 1 = update PC, 0 = hold (stall).
    std::array<unsigned int, 1> WriteEnable_Internal;

    // --- Input Ports ---
    /// Input source for the next PC value (e.g., from PC+inc, branch target, jump target).
    InputSliceSource<24> NextValueIn_Src{ "NextValueIn_Src" };
    /// Input source for the write enable signal (e.g., from Hazard Detection Unit).
    InputSliceSource<1> WriteEnable_Src{ "WriteEnable_Src" };

    // --- Output Ports ---
    /// Output target 1 for the current PC value (e.g., to Memory's instruction address input).
    OutputSliceTarget<24> OUT1{ "OUT1" };
    /// Output target 2 for the current PC value (e.g., to an adder for PC+increment calculation).
    OutputSliceTarget<24> OUT2{ "OUT2" };

    /**
     * @brief Constructor for the ProgramCounter component.
     * Initializes port names, sets PC to 0, and enables writing by default.
     * @param name The name of this ProgramCounter instance.
     */
    ProgramCounter(const std::string& name = "PC") : DriveableUnit(name) {
        // Initialize input and output ports with parent component information.
        NextValueIn_Src.initParent(this);
        WriteEnable_Src.initParent(this);
        OUT1.initParent(this);
        OUT2.initParent(this);

        // Initialize internal state.
        Value.fill(0); // PC starts at address 0.
        NextValueIn_Internal.fill(0);
        WriteEnable_Internal[0] = 1; // PC writing is enabled by default (no stall).
    }
    ~ProgramCounter() override = default; // Default destructor.

    /**
     * @brief Simulates the clock edge update for the PC.
     * Reads the next PC value and write enable signal. If enabled, updates
     * the PC's `Value` with `NextValueIn_Internal`.
     * @param cycle The current simulation cycle number (for debugging).
     */
    void DriveUnit(int cycle = 0) override {
        // 1. Read inputs that determine the PC's next state.
        NextValueIn_Src.read(NextValueIn_Internal);
        WriteEnable_Src.read(WriteEnable_Internal);

        // 2. Update PC state if write is enabled.
        if (WriteEnable_Internal[0] == 1) {
            Value = NextValueIn_Internal; // Latch the new PC value.
        }
        // If WriteEnable_Internal[0] is 0, the PC is stalled and Value remains unchanged.

        // Debug print for cycle 8.
        if (cycle == 8)
        {
            std::cout << component_name << " Value = " << Convert::ArrayToUInt(Value)
                << " NextValueIn_Internal = " << Convert::ArrayToUInt(NextValueIn_Internal)
                << " WriteEnable_Internal = " << Convert::ArrayToUInt(WriteEnable_Internal) << std::endl;
        }
    }

    /**
     * @brief Writes the current PC `Value` to its configured output targets.
     * This typically occurs during the combinational phase of a cycle, making the
     * current PC available for fetching and next PC calculation.
     */
    void WriteOutput() override {
        // Propagate the current PC value to both output ports.
        OUT1.write(Value);
        OUT2.write(Value);
    }
};

/**
 * @brief Implements the Hazard Detection Unit for the MIPS pipeline.
 *
 * Specifically detects load-use hazards where an instruction in the ID stage
 * requires a result from a `lw` (load word) instruction currently in the EX stage.
 * If such a hazard is detected, it asserts control signals to stall the pipeline
 * (PC and IF/ID register) and insert a NOP into the ID/EX register.
 */
class HazardDetector : public DriveableUnit {
public:
    // --- State: Internal buffers for inputs read from pipeline registers ---
    /// MemRead control signal of the instruction in the ID/EX register (indicates if it's a load).
    std::array<unsigned int, 1> ID_EX_MemRead_Internal;
    /// Rt register number (destination for lw) of the instruction in the ID/EX register.
    std::array<unsigned int, 4> ID_EX_Rt_Internal;

    /// Rs register number (source operand 1) of the instruction in the IF/ID register.
    std::array<unsigned int, 4> IF_ID_Rs_Internal;
    /// Rt register number (source operand 2) of the instruction in the IF/ID register.
    std::array<unsigned int, 4> IF_ID_Rt_Internal;

    // --- State: Internal buffers for output control signals ---
    /// Output signal to control PC write enable (0 to stall, 1 to enable).
    std::array<unsigned int, 1> PC_WRITE_OUTPUT_Internal;
    /// Output signal to control IF/ID register write enable (0 to stall, 1 to enable).
    std::array<unsigned int, 1> IF_ID_WRITE_OUTPUT_Internal;
    /// Output signal to select NOP controls for the ID/EX mux (1 for NOP, 0 for normal).
    std::array<unsigned int, 1> CONTROL_MUX_SELECT_OUTPUT_Internal;

    // --- Input Ports ---
    /// Source for MemRead control bit from ID/EX register (M controls).
    InputSliceSource<1> ID_EX_MemRead_Src{ "ID_EX_MemRead_Src" };
    /// Source for Rt register number from ID/EX register.
    InputSliceSource<4> ID_EX_Rt_Src{ "ID_EX_Rt_Src" };

    /// Source for Rs register number from IF/ID register.
    InputSliceSource<4> IF_ID_Rs_Src{ "IF_ID_Rs_Src" };
    /// Source for Rt register number from IF/ID register.
    InputSliceSource<4> IF_ID_Rt_Src{ "IF_ID_Rt_Src" };

    // --- Output Ports ---
    /// Target for PC write enable signal (to ProgramCounter).
    OutputSliceTarget<1> PC_WRITE_ENABLE_OUTPUT{ "PC_WRITE_ENABLE_OUTPUT" };
    /// Target for IF/ID register write enable signal (to PipelineRegisters).
    OutputSliceTarget<1> IF_ID_WRITE_ENABLE_OUTPUT{ "IF_ID_WRITE_ENABLE_OUTPUT" };
    /// Target for control mux selector (to ClearControlLinesMultiplexor).
    OutputSliceTarget<1> CONTROL_MUX_SELECTOR_OUTPUT{ "CONTROL_MUX_SELECTOR_OUTPUT" };

    /**
     * @brief Constructor for the HazardDetector component.
     * Initializes port names and default output states (no stall).
     * @param name The name of this HazardDetector instance.
     */
    HazardDetector(const std::string& name = "HazardDetector") : DriveableUnit(name) {
        // Initialize all input and output ports with parent component information.
        ID_EX_MemRead_Src.initParent(this);
        ID_EX_Rt_Src.initParent(this);
        IF_ID_Rs_Src.initParent(this);
        IF_ID_Rt_Src.initParent(this);
        PC_WRITE_ENABLE_OUTPUT.initParent(this);
        IF_ID_WRITE_ENABLE_OUTPUT.initParent(this);
        CONTROL_MUX_SELECTOR_OUTPUT.initParent(this);

        // Initialize internal buffers for inputs.
        ID_EX_MemRead_Internal.fill(0);
        ID_EX_Rt_Internal.fill(0);
        IF_ID_Rs_Internal.fill(0);
        IF_ID_Rt_Internal.fill(0);
        // Default output signals to "no hazard" / "no stall" state.
        PC_WRITE_OUTPUT_Internal[0] = 1;       // PC write enabled.
        IF_ID_WRITE_OUTPUT_Internal[0] = 1;    // IF/ID latch enabled.
        CONTROL_MUX_SELECT_OUTPUT_Internal[0] = 0; // Normal controls (not NOP).
    }
    ~HazardDetector() override = default; // Default destructor.

    /**
     * @brief Simulates hazard detection logic for one cycle.
     * Checks for a load-use hazard between an instruction in ID/EX (load)
     * and an instruction in IF/ID (using the load's result).
     * Sets stall and NOP control signals if a hazard is detected.
     * @param cycle The current simulation cycle number (for debugging).
     */
    void DriveUnit(int cycle = 0) override {
        // 1. Read necessary fields from pipeline registers into internal buffers.
        //    These fields are from the instruction currently in EX (ID_EX_*) and ID (IF_ID_*).
        //    Inputs are sourced from the stable outputs of ID/EX and IF/ID registers.
        ID_EX_MemRead_Src.read(ID_EX_MemRead_Internal); // Is instruction in EX a load? (MemRead signal)
        ID_EX_Rt_Src.read(ID_EX_Rt_Internal);           // Destination register (Rt) of the potential load in EX.
        IF_ID_Rs_Src.read(IF_ID_Rs_Internal);           // Rs operand of the instruction currently in ID.
        IF_ID_Rt_Src.read(IF_ID_Rt_Internal);           // Rt operand of the instruction currently in ID.

        // Define a 4-bit representation of register $zero for comparison.
        const std::array<unsigned int, 4> zeroReg = { 0, 0, 0, 0 };

        // 2. --- Load-Use Hazard Detection Logic ---
        // A stall is required if:
        //   a) The instruction in the EX stage (from ID/EX register) is a `lw` (ID_EX_MemRead_Internal[0] == 1).
        //   b) The destination register of this `lw` (ID_EX_Rt_Internal) is *not* $r0.
        //   c) This destination register (ID_EX_Rt_Internal) is used as a source operand (Rs or Rt)
        //      by the instruction currently in the ID stage (from IF/ID register).
        // This specific hazard cannot be fully resolved by forwarding to the ID stage in time
        // for ALU operations or branch comparisons that happen early in ID.

        bool isLoadInEX = (ID_EX_MemRead_Internal[0] == 1);
        // Check if the load's destination register is not $zero. Writes to $zero are no-ops.
        bool loadDestNotZero = !std::equal(ID_EX_Rt_Internal.begin(), ID_EX_Rt_Internal.end(), zeroReg.begin());

        bool idNeedsLoadResult = false;
        if (isLoadInEX && loadDestNotZero) { // Only check for dependency if it's a load writing to a non-zero register.
            // Check if IF/ID.Rs needs the result from ID/EX.Rt
            bool useAsRs = std::equal(ID_EX_Rt_Internal.begin(), ID_EX_Rt_Internal.end(), IF_ID_Rs_Internal.begin());
            // Check if IF/ID.Rt needs the result from ID/EX.Rt
            bool useAsRt = std::equal(ID_EX_Rt_Internal.begin(), ID_EX_Rt_Internal.end(), IF_ID_Rt_Internal.begin());
            idNeedsLoadResult = useAsRs || useAsRt;
        }

        // The stall condition: a load in EX whose result is needed by the instruction in ID.
        bool stall_pipeline = idNeedsLoadResult; // Simplified: idNeedsLoadResult already implies isLoadInEX & loadDestNotZero

        // 3. Set output control signals based on hazard detection.
        if (stall_pipeline) {
            PC_WRITE_OUTPUT_Internal[0] = 0;          // Stall PC update.
            IF_ID_WRITE_OUTPUT_Internal[0] = 0;       // Stall IF/ID register (prevents new instr from entering ID).
            CONTROL_MUX_SELECT_OUTPUT_Internal[0] = 1; // Force NOP controls into ID/EX for the stalled instr.
        }
        else {
            PC_WRITE_OUTPUT_Internal[0] = 1;          // No stall, PC updates normally.
            IF_ID_WRITE_OUTPUT_Internal[0] = 1;       // No stall, IF/ID latches normally.
            CONTROL_MUX_SELECT_OUTPUT_Internal[0] = 0; // Normal controls pass to ID/EX.
        }

        // Debug Print for cycle 8.
        if (cycle == 8)
        {
            std::cout << "Hazard Detector:" << std::endl;
            std::cout << "  Inputs: EX_is_LW=" << isLoadInEX
                << " EX_LW_Dest=" << Convert::ArrayToUInt(ID_EX_Rt_Internal)
                << " | ID_Needs: Rs=" << Convert::ArrayToUInt(IF_ID_Rs_Internal)
                << " Rt=" << Convert::ArrayToUInt(IF_ID_Rt_Internal) << std::endl;
            std::cout << "  Result: Stall=" << stall_pipeline
                << " (PC_Wr=" << PC_WRITE_OUTPUT_Internal[0]
                << " IFID_Wr=" << IF_ID_WRITE_OUTPUT_Internal[0]
                << " NOP_Ctrl=" << CONTROL_MUX_SELECT_OUTPUT_Internal[0] << ")" << std::endl;
            std::cout << " ID_EX_MemRead_Internal="; Convert::PrintArray(ID_EX_MemRead_Internal); // Print the actual array
            std::cout << std::endl;
        }
    }

    /**
     * @brief Writes the calculated stall/NOP control signals to their output targets.
     */
    void WriteOutput() override {
        PC_WRITE_ENABLE_OUTPUT.write(PC_WRITE_OUTPUT_Internal);
        IF_ID_WRITE_ENABLE_OUTPUT.write(IF_ID_WRITE_OUTPUT_Internal);
        CONTROL_MUX_SELECTOR_OUTPUT.write(CONTROL_MUX_SELECT_OUTPUT_Internal);
    }
};

/**
 * @brief Compares two 24-bit data values for equality.
 *
 * Used in the ID stage for MIPS branch instructions like `beq` and `bne`.
 * Takes two 24-bit inputs (typically from register read ports or forwarding paths)
 * and outputs a 1-bit signal: 1 if inputs are equal, 0 otherwise.
 */
class BranchComparator : public DriveableUnit {
public:
    // --- State Variables ---
    /// Internal buffer for the first 24-bit data input.
    std::array<unsigned int, 24> ReadData1_Internal;
    /// Internal buffer for the second 24-bit data input.
    std::array<unsigned int, 24> ReadData2_Internal;
    /// Internal buffer for the 1-bit equality result (1 if equal, 0 if not).
    std::array<unsigned int, 1> EQUAL_OUTPUT_Internal;

    // --- Input Ports ---
    /// Input source for the first 24-bit operand (e.g., Rs value).
    InputSliceSource<24> ReadData1_Src{ "ReadData1_Src" };
    /// Input source for the second 24-bit operand (e.g., Rt value).
    InputSliceSource<24> ReadData2_Src{ "ReadData2_Src" };

    // --- Output Ports ---
    /// Output target for the 1-bit equality signal.
    OutputSliceTarget<1> EQUAL_OUTPUT{ "EQUAL_OUTPUT" };

    /**
     * @brief Constructor for the BranchComparator component.
     * Initializes port names and internal buffers.
     * @param name The name of this BranchComparator instance.
     */
    BranchComparator(const std::string& name = "BranchComparator") : DriveableUnit(name) {
        // Initialize input and output ports with parent component information.
        ReadData1_Src.initParent(this);
        ReadData2_Src.initParent(this);
        EQUAL_OUTPUT.initParent(this);

        // Initialize internal state buffers.
        ReadData1_Internal.fill(0);
        ReadData2_Internal.fill(0);
        EQUAL_OUTPUT_Internal[0] = 1; // Default to equal (arbitrary, recalculated each cycle).
    }
    ~BranchComparator() override = default; // Default destructor.

    /**
     * @brief Simulates the comparison logic for one cycle.
     * Reads the two 24-bit data inputs and sets EQUAL_OUTPUT_Internal
     * based on their equality.
     * @param cycle The current simulation cycle number (for debugging).
     */
    void DriveUnit(int cycle = 0) override {
        // 1. Read data inputs into internal buffers.
        ReadData1_Src.read(ReadData1_Internal);
        ReadData2_Src.read(ReadData2_Internal);

        // 2. Compare the two 24-bit arrays for equality.
        //    std::equal compares elements in two ranges.
        bool are_equal = std::equal(ReadData1_Internal.begin(), ReadData1_Internal.end(), ReadData2_Internal.begin());
        EQUAL_OUTPUT_Internal[0] = are_equal ? 1 : 0; // Set output to 1 if equal, 0 otherwise.

        // Debug print for cycle 8.
        if (cycle == 8)
        {
            std::cout << component_name << " ReadData1_Internal = " << Convert::ArrayToUInt(ReadData1_Internal)
                << " ReadData2_Internal = " << Convert::ArrayToUInt(ReadData2_Internal)
                << " EQUAL_OUTPUT_Internal = " << Convert::ArrayToUInt(EQUAL_OUTPUT_Internal) << std::endl;
        }
    }

    /**
     * @brief Writes the 1-bit equality result to its output target.
     */
    void WriteOutput() override {
        EQUAL_OUTPUT.write(EQUAL_OUTPUT_Internal);
    }
};

/**
 * @brief Implements the Forwarding Unit (or Bypass Unit) for the MIPS pipeline.
 *
 * Detects data hazards by comparing source register requirements of instructions
 * in earlier stages (ID, EX) with destination registers of instructions in later
 * stages (EX, MEM, WB).
 * Generates selector signals for multiplexers to forward data from later stages
 * directly to the inputs of earlier stages (ALU inputs in EX, Comparator inputs in ID),
 * bypassing the register file read for data that is not yet written back.
 */
class Forwarder : public DriveableUnit {
public:
    // --- State for ALU Forwarding (Instruction in EX stage) ---
    /// Rs register number needed by the instruction currently in EX stage.
    std::array<unsigned int, 4> ID_EX_Rs_Internal;
    /// Rt register number needed by the instruction currently in EX stage.
    std::array<unsigned int, 4> ID_EX_Rt_Internal;
    /// 2-bit selector signal for the ALU's Input A forwarding mux (ALUForwardAMux).
    std::array<unsigned int, 2> ForwardA_EX_Internal;
    /// 2-bit selector signal for the ALU's Input B forwarding mux (ALUForwardBMux).
    std::array<unsigned int, 2> ForwardB_EX_Internal;

    // --- State for ID Stage Branch Comparator Forwarding ---
    /// Rs register number needed by the instruction currently in ID stage.
    std::array<unsigned int, 4> IF_ID_Rs_Internal;
    /// Rt register number needed by the instruction currently in ID stage.
    std::array<unsigned int, 4> IF_ID_Rt_Internal;
    /// 2-bit selector signal for Branch Comparator's Input A forwarding mux (ID_ForwardAMux4Way).
    std::array<unsigned int, 2> ForwardA_ID_Internal;
    /// 2-bit selector signal for Branch Comparator's Input B forwarding mux (ID_ForwardBMux4Way).
    std::array<unsigned int, 2> ForwardB_ID_Internal;

    // --- State from Pipeline Registers for Hazard Detection ---
    /// Destination register number of the instruction currently in EX stage (from ID/EX).
    std::array<unsigned int, 4> ID_EX_WriteRegister_Internal;
    /// RegWrite control signal of the instruction currently in EX stage (from ID/EX).
    std::array<unsigned int, 1> ID_EX_RegWrite_Internal;

    /// Destination register number of the instruction in the EX/MEM register.
    std::array<unsigned int, 4> EX_MEM_WriteRegister_Internal;
    /// RegWrite control signal of the instruction in the EX/MEM register.
    std::array<unsigned int, 1> EX_MEM_RegWrite_Internal;

    /// Destination register number of the instruction in the MEM/WB register.
    std::array<unsigned int, 4> MEM_WB_WriteRegister_Internal;
    /// RegWrite control signal of the instruction in the MEM/WB register.
    std::array<unsigned int, 1> MEM_WB_RegWrite_Internal;


    // --- Input Ports ---
    // For ALU Forwarding (info about instruction currently in EX stage, from ID/EX register)
    /// Source for Rs# of instruction in EX stage.
    InputSliceSource<4> ID_EX_Rs_Src{ "ID_EX_Rs_Src" };
    /// Source for Rt# of instruction in EX stage.
    InputSliceSource<4> ID_EX_Rt_Src{ "ID_EX_Rt_Src" };

    // For ID Stage Forwarding (info about instruction currently in ID stage, from IF/ID register)
    /// Source for Rs# of instruction in ID stage.
    InputSliceSource<4> IF_ID_Rs_Src{ "IF_ID_Rs_Src" };
    /// Source for Rt# of instruction in ID stage.
    InputSliceSource<4> IF_ID_Rt_Src{ "IF_ID_Rt_Src" };

    // For ID Stage Forwarding (info about destination of instruction currently in EX stage)
    /// Source for destination register# of instruction in EX (e.g., from WriteRegisterMux output).
    InputSliceSource<4> ID_EX_WriteRegister_Src{ "ID_EX_WriteRegister_Src" };
    /// Source for RegWrite signal of instruction in EX (from ID/EX control bits).
    InputSliceSource<1> ID_EX_RegWrite_Src{ "ID_EX_RegWrite_Src" };

    // Common Inputs (from EX/MEM and MEM/WB pipeline registers)
    /// Source for destination register# from EX/MEM register.
    InputSliceSource<4> EX_MEM_WriteRegister_Src{ "EX_MEM_WriteRegister_Src" };
    /// Source for RegWrite signal from EX/MEM register.
    InputSliceSource<1> EX_MEM_RegWrite_Src{ "EX_MEM_RegWrite_Src" };
    /// Source for destination register# from MEM/WB register.
    InputSliceSource<4> MEM_WB_WriteRegister_Src{ "MEM_WB_WriteRegister_Src" };
    /// Source for RegWrite signal from MEM/WB register.
    InputSliceSource<1> MEM_WB_RegWrite_Src{ "MEM_WB_RegWrite_Src" };

    // --- Output Ports ---
    /// Target for the ALU Input A forwarding mux selector.
    OutputSliceTarget<2> ALU_FORWARD_A_MUX_SELECTOR{ "ALU_FORWARD_A_MUX_SELECTOR" };
    /// Target for the ALU Input B forwarding mux selector.
    OutputSliceTarget<2> ALU_FORWARD_B_MUX_SELECTOR{ "ALU_FORWARD_B_MUX_SELECTOR" };
    /// Target for the Branch Comparator Input A forwarding mux selector (4-way).
    OutputSliceTarget<2> ID_FORWARD_A_MUX_SELECTOR{ "ID_FORWARD_A_MUX_SELECTOR" };
    /// Target for the Branch Comparator Input B forwarding mux selector (4-way).
    OutputSliceTarget<2> ID_FORWARD_B_MUX_SELECTOR{ "ID_FORWARD_B_MUX_SELECTOR" };

    // --- Forwarding Codes for Mux Control ---
    // For ID Stage 4-way Comparator Muxes (ID_ForwardAMux4Way, ID_ForwardBMux4Way)
    // These determine which data source is selected for the branch comparator inputs.
    // LSB-first: 00, 01, 10, 11
    const unsigned int ID_FWD_REGFILE = 0; ///< 00: Select data from Register File (no forward).
    const unsigned int ID_FWD_EX_ALU = 1; ///< 01: Forward result from current ALU output (instruction in EX).
    const unsigned int ID_FWD_MEM = 2; ///< 10: Forward result from EX/MEM pipeline register (instruction in MEM).
    const unsigned int ID_FWD_WB = 3; ///< 11: Forward result from MEM/WB pipeline register (instruction in WB).

    // For EX Stage 3-way ALU Input Muxes (ALUForwardAMux, ALUForwardBMux)
    // These determine which data source is selected for the ALU's main operands.
    // LSB-first: 00, 01, 10
    const unsigned int EX_FWD_IDEX = 0; ///< 00: Select data from ID/EX register (originally from RegFile).
    const unsigned int EX_FWD_MEMWB = 1; ///< 01: Forward result from MEM/WB pipeline register.
    const unsigned int EX_FWD_EXMEM = 2; ///< 10: Forward result from EX/MEM pipeline register.

    /**
     * @brief Constructor for the Forwarder component.
     * Initializes port names and internal buffers.
     * @param name The name of this Forwarder instance.
     */
    Forwarder(const std::string& name = "Forwarder") : DriveableUnit(name) {
        // Initialize all input and output ports.
        ID_EX_Rs_Src.initParent(this);
        ID_EX_Rt_Src.initParent(this);
        IF_ID_Rs_Src.initParent(this);
        IF_ID_Rt_Src.initParent(this);
        ID_EX_WriteRegister_Src.initParent(this);
        ID_EX_RegWrite_Src.initParent(this);
        EX_MEM_WriteRegister_Src.initParent(this);
        MEM_WB_WriteRegister_Src.initParent(this);
        EX_MEM_RegWrite_Src.initParent(this);
        MEM_WB_RegWrite_Src.initParent(this);
        ALU_FORWARD_A_MUX_SELECTOR.initParent(this);
        ALU_FORWARD_B_MUX_SELECTOR.initParent(this);
        ID_FORWARD_A_MUX_SELECTOR.initParent(this);
        ID_FORWARD_B_MUX_SELECTOR.initParent(this);

        // Initialize internal state buffers.
        ID_EX_Rs_Internal.fill(0);
        ID_EX_Rt_Internal.fill(0);
        IF_ID_Rs_Internal.fill(0);
        IF_ID_Rt_Internal.fill(0);
        ID_EX_WriteRegister_Internal.fill(0);
        ID_EX_RegWrite_Internal[0] = 0;
        EX_MEM_WriteRegister_Internal.fill(0);
        MEM_WB_WriteRegister_Internal.fill(0);
        EX_MEM_RegWrite_Internal[0] = 0;
        MEM_WB_RegWrite_Internal[0] = 0;
        ForwardA_EX_Internal.fill(0); // Default to EX_FWD_IDEX
        ForwardB_EX_Internal.fill(0); // Default to EX_FWD_IDEX
        ForwardA_ID_Internal.fill(0); // Default to ID_FWD_REGFILE
        ForwardB_ID_Internal.fill(0); // Default to ID_FWD_REGFILE
    }
    ~Forwarder() override = default; // Default destructor.

    /**
     * @brief Simulates forwarding logic for one cycle.
     * Reads register numbers and control signals from various pipeline stages.
     * Determines if forwarding is needed for ALU inputs (EX stage) and
     * branch comparator inputs (ID stage). Sets the appropriate mux selector outputs.
     * @param cycle The current simulation cycle number (for debugging).
     */
    void DriveUnit(int cycle = 0) override {
        // 1. Read all necessary inputs from pipeline registers and control signals.
        //    These inputs are sourced from the stable outputs of the respective pipeline registers.
        ID_EX_Rs_Src.read(ID_EX_Rs_Internal); // Rs# of instruction currently in EX.
        ID_EX_Rt_Src.read(ID_EX_Rt_Internal); // Rt# of instruction currently in EX.
        IF_ID_Rs_Src.read(IF_ID_Rs_Internal); // Rs# of instruction currently in ID.
        IF_ID_Rt_Src.read(IF_ID_Rt_Internal); // Rt# of instruction currently in ID.

        // Info about instruction currently in EX (its destination and RegWrite signal).
        ID_EX_WriteRegister_Src.read(ID_EX_WriteRegister_Internal);
        ID_EX_RegWrite_Src.read(ID_EX_RegWrite_Internal);

        // Info about instruction currently in MEM (its destination and RegWrite signal from EX/MEM).
        EX_MEM_WriteRegister_Src.read(EX_MEM_WriteRegister_Internal);
        EX_MEM_RegWrite_Src.read(EX_MEM_RegWrite_Internal);

        // Info about instruction currently in WB (its destination and RegWrite signal from MEM/WB).
        MEM_WB_WriteRegister_Src.read(MEM_WB_WriteRegister_Internal);
        MEM_WB_RegWrite_Src.read(MEM_WB_RegWrite_Internal);

        // Constant for comparing against register $zero (index 0).
        const std::array<unsigned int, 4> zeroReg = { 0, 0, 0, 0 };

        // 2. Determine if instructions in later stages are writing to a non-zero register.
        bool id_ex_is_writing_to_nonzero_reg = (ID_EX_RegWrite_Internal[0] == 1) &&
            !std::equal(ID_EX_WriteRegister_Internal.begin(), ID_EX_WriteRegister_Internal.end(), zeroReg.begin());
        bool ex_mem_is_writing_to_nonzero_reg = (EX_MEM_RegWrite_Internal[0] == 1) &&
            !std::equal(EX_MEM_WriteRegister_Internal.begin(), EX_MEM_WriteRegister_Internal.end(), zeroReg.begin());
        bool mem_wb_is_writing_to_nonzero_reg = (MEM_WB_RegWrite_Internal[0] == 1) &&
            !std::equal(MEM_WB_WriteRegister_Internal.begin(), MEM_WB_WriteRegister_Internal.end(), zeroReg.begin());

        // 3. --- ALU Forwarding Logic (for instruction in EX stage) ---
        // Determines selector for ALUForwardAMux and ALUForwardBMux (3-way muxes).
        // These muxes select inputs for the ALU in the EX stage.
        unsigned int forward_a_ex_code = EX_FWD_IDEX; // Default: use operand from ID/EX register.
        unsigned int forward_b_ex_code = EX_FWD_IDEX;

        // Priority 1: Forward from EX/MEM to EX stage ALU input.
        // If instr in MEM is writing a non-zero reg, and EX needs its result.
        if (ex_mem_is_writing_to_nonzero_reg) {
            if (std::equal(EX_MEM_WriteRegister_Internal.begin(), EX_MEM_WriteRegister_Internal.end(), ID_EX_Rs_Internal.begin())) {
                forward_a_ex_code = EX_FWD_EXMEM; // Forward EX/MEM.ALUResult to ALU Input A.
            }
            if (std::equal(EX_MEM_WriteRegister_Internal.begin(), EX_MEM_WriteRegister_Internal.end(), ID_EX_Rt_Internal.begin())) {
                forward_b_ex_code = EX_FWD_EXMEM; // Forward EX/MEM.ALUResult to ALU Input B.
            }
        }

        // Priority 2: Forward from MEM/WB to EX stage ALU input.
        // (Only if not already forwarding from EX/MEM for that same operand).
        if (mem_wb_is_writing_to_nonzero_reg) {
            if (forward_a_ex_code == EX_FWD_IDEX && // If not already forwarding from EX/MEM for A
                std::equal(MEM_WB_WriteRegister_Internal.begin(), MEM_WB_WriteRegister_Internal.end(), ID_EX_Rs_Internal.begin())) {
                forward_a_ex_code = EX_FWD_MEMWB; // Forward MEM/WB result to ALU Input A.
            }
            if (forward_b_ex_code == EX_FWD_IDEX && // If not already forwarding from EX/MEM for B
                std::equal(MEM_WB_WriteRegister_Internal.begin(), MEM_WB_WriteRegister_Internal.end(), ID_EX_Rt_Internal.begin())) {
                forward_b_ex_code = EX_FWD_MEMWB; // Forward MEM/WB result to ALU Input B.
            }
        }
        ForwardA_EX_Internal = Convert::ToArray2Bit(forward_a_ex_code);
        ForwardB_EX_Internal = Convert::ToArray2Bit(forward_b_ex_code);

        // 4. --- Branch Comparator Forwarding Logic (for instruction in ID stage) ---
        // Determines selector for ID_ForwardAMux4Way and ID_ForwardBMux4Way.
        // These muxes select inputs for the Branch Comparator in the ID stage.
        unsigned int forward_a_id_code = ID_FWD_REGFILE; // Default: use operand from Register File.
        unsigned int forward_b_id_code = ID_FWD_REGFILE;

        // Priority 1: Forward from current ALU output (EX stage) to ID stage comparator.
        // If instr in EX is writing a non-zero reg, and ID needs its result.
        if (id_ex_is_writing_to_nonzero_reg) {
            if (std::equal(ID_EX_WriteRegister_Internal.begin(), ID_EX_WriteRegister_Internal.end(), IF_ID_Rs_Internal.begin())) {
                forward_a_id_code = ID_FWD_EX_ALU; // Forward current ALU result to Comparator Input A.
            }
            if (std::equal(ID_EX_WriteRegister_Internal.begin(), ID_EX_WriteRegister_Internal.end(), IF_ID_Rt_Internal.begin())) {
                forward_b_id_code = ID_FWD_EX_ALU; // Forward current ALU result to Comparator Input B.
            }
        }

        // Priority 2: Forward from EX/MEM to ID stage comparator.
        // (Only if not already forwarding from current ALU output for that same operand).
        if (ex_mem_is_writing_to_nonzero_reg) {
            if (forward_a_id_code == ID_FWD_REGFILE && // If not already forwarding from EX_ALU for A
                std::equal(EX_MEM_WriteRegister_Internal.begin(), EX_MEM_WriteRegister_Internal.end(), IF_ID_Rs_Internal.begin())) {
                forward_a_id_code = ID_FWD_MEM; // Forward EX/MEM.ALUResult to Comparator Input A.
            }
            if (forward_b_id_code == ID_FWD_REGFILE && // If not already forwarding from EX_ALU for B
                std::equal(EX_MEM_WriteRegister_Internal.begin(), EX_MEM_WriteRegister_Internal.end(), IF_ID_Rt_Internal.begin())) {
                forward_b_id_code = ID_FWD_MEM; // Forward EX/MEM.ALUResult to Comparator Input B.
            }
        }

        // Priority 3: Forward from MEM/WB to ID stage comparator.
        // (Only if not already forwarding from a closer stage for that same operand).
        if (mem_wb_is_writing_to_nonzero_reg) {
            if (forward_a_id_code == ID_FWD_REGFILE && // If not already forwarding from EX_ALU or EX/MEM for A
                std::equal(MEM_WB_WriteRegister_Internal.begin(), MEM_WB_WriteRegister_Internal.end(), IF_ID_Rs_Internal.begin())) {
                forward_a_id_code = ID_FWD_WB; // Forward MEM/WB result to Comparator Input A.
            }
            if (forward_b_id_code == ID_FWD_REGFILE && // If not already forwarding from EX_ALU or EX/MEM for B
                std::equal(MEM_WB_WriteRegister_Internal.begin(), MEM_WB_WriteRegister_Internal.end(), IF_ID_Rt_Internal.begin())) {
                forward_b_id_code = ID_FWD_WB; // Forward MEM/WB result to Comparator Input B.
            }
        }
        ForwardA_ID_Internal = Convert::ToArray2Bit(forward_a_id_code);
        ForwardB_ID_Internal = Convert::ToArray2Bit(forward_b_id_code);

        // Debug print block.
        if (cycle == 8 || (cycle >= 2 && cycle <= 4 && component_name == "Forwarder")) // More targeted debug
        {
            std::cout << "Forwarding Unit (Cycle " << cycle << "):" << std::endl;
            std::cout << "  EX Stage Fwd (to ALU Muxes):";
            std::cout << " A_EX_sel=" << forward_a_ex_code << " (for ID/EX.Rs#=" << Convert::ArrayToUInt(ID_EX_Rs_Internal) << ")";
            std::cout << ", B_EX_sel=" << forward_b_ex_code << " (for ID/EX.Rt#=" << Convert::ArrayToUInt(ID_EX_Rt_Internal) << ")" << std::endl;

            std::cout << "  ID Stage Fwd (to Comparator Muxes):";
            std::cout << " A_ID_sel=" << forward_a_id_code << " (for IF/ID.Rs#=" << Convert::ArrayToUInt(IF_ID_Rs_Internal) << ")";
            std::cout << ", B_ID_sel=" << forward_b_id_code << " (for IF/ID.Rt#=" << Convert::ArrayToUInt(IF_ID_Rt_Internal) << ")" << std::endl;

            std::cout << "  Source Info: ID/EX(WrReg#=" << Convert::ArrayToUInt(ID_EX_WriteRegister_Internal) << ",RegWr=" << ID_EX_RegWrite_Internal[0] << ")"
                << " | EX/MEM(WrReg#=" << Convert::ArrayToUInt(EX_MEM_WriteRegister_Internal) << ",RegWr=" << EX_MEM_RegWrite_Internal[0] << ")"
                << " | MEM/WB(WrReg#=" << Convert::ArrayToUInt(MEM_WB_WriteRegister_Internal) << ",RegWr=" << MEM_WB_RegWrite_Internal[0] << ")"
                << std::endl;
        }
    }

    /**
     * @brief Writes the calculated forwarding mux selector signals to their output targets.
     */
    void WriteOutput() override {
        ALU_FORWARD_A_MUX_SELECTOR.write(ForwardA_EX_Internal);
        ALU_FORWARD_B_MUX_SELECTOR.write(ForwardB_EX_Internal);
        ID_FORWARD_A_MUX_SELECTOR.write(ForwardA_ID_Internal);
        ID_FORWARD_B_MUX_SELECTOR.write(ForwardB_ID_Internal);
    }
};

/**
 * @brief Implements the main Control Unit of the MIPS processor.
 *
 * Decodes the opcode of an instruction (from IF/ID register) to generate
 * various control signals for other pipeline stages (EX, MEM, WB).
 * These signals dictate operations like register destination, ALU source,
 * memory read/write, register write enable, ALU operation type, PC source selection,
 * and jump control.
 */
class Controller : public DriveableUnit {
public:
    // --- State ---
    /// Internal buffer for the 4-bit opcode from the instruction.
    std::array<unsigned int, 4> Opcode_Internal;

    // --- Internal Buffers for Control Signals Generated ---
    // These are set based on Opcode_Internal.
    /// EX stage: RegDst (selects Rd or Rt as destination for R-type vs I-type).
    std::array<unsigned int, 1> RegDst_Internal{ 0 };
    /// EX stage: ALUSrc (selects ALU input B: register or immediate).
    std::array<unsigned int, 1> ALUSrc_Internal{ 0 };
    /// WB stage: MemToReg (selects data for register write: ALU result or memory data).
    std::array<unsigned int, 1> MemToReg_Internal{ 0 };
    /// WB stage: RegWrite (enables writing to the register file).
    std::array<unsigned int, 1> RegWrite_Internal{ 0 };
    /// MEM stage: MemRead (enables reading from data memory).
    std::array<unsigned int, 1> MemRead_Internal{ 0 };
    /// MEM stage: MemWrite (enables writing to data memory).
    std::array<unsigned int, 1> MemWrite_Internal{ 0 };
    /// EX stage: ALUOp (2-bit signal to ALUController, partially defines ALU operation).
    std::array<unsigned int, 2> ALUOp_Internal{ 0, 0 };
    /// ID stage: PCSource (Branch signal for PC mux, 1 if branch condition might lead to taken branch).
    std::array<unsigned int, 1> PCSource_Internal{ 0 };
    /// ID stage: Jump_IF_Flush (Asserted for J-type, signals IF/ID flush).
    std::array<unsigned int, 1> Jump_IF_Flush_Internal{ 0 };
    /// ID stage: Jump (Asserted for J-type, selects Jump Target Mux).
    std::array<unsigned int, 1> Jump_Internal{ 0 };

    /// Combined 8-bit control signal bundle primarily for the ID/EX pipeline register.
    /// Layout (LSB first): [0:ALUSrc] [1-2:ALUOp] [3:RegDst] [4:MemWrite] [5:MemRead] [6:MemToReg] [7:RegWrite]
    std::array<unsigned int, 8> Combined_Controls_Output_Internal{ 0 };

    // --- Input Ports ---
    /// Input source for the 4-bit Opcode (typically from IF/ID register, instruction bits 20-23).
    InputSliceSource<4> Opcode_Src{ "Opcode_Src" };

    // --- Output Ports ---
    /// Output target for PCSource (Branch) signal (to Branch AND Gate).
    OutputSliceTarget<1> PC_SOURCE_OUTPUT{ "PC_SOURCE_OUTPUT" };
    /// Output target for Jump IF_Flush signal (to Flush OR Gate for PipelineRegisters).
    OutputSliceTarget<1> JUMP_IF_FLUSH_OUTPUT{ "JUMP_IF_FLUSH_OUTPUT" };
    /// Output target for combined EX/MEM/WB control lines (to mux feeding ID/EX_Next_Input).
    OutputSliceTarget<8> CONTROL_LINES_OUTPUT{ "CONTROL_LINES_OUTPUT" };
    /// Output target for Jump signal (to Jump Multiplexer selector).
    OutputSliceTarget<1> JUMP_OUTPUT{ "JUMP_OUTPUT" };

    /**
     * @brief Constructor for the Controller.
     * Initializes port names and internal control signals to default (NOP-like) states.
     * @param name The name of this Controller instance.
     */
    Controller(const std::string& name = "Controller") : DriveableUnit(name) {
        // Initialize input and output ports.
        Opcode_Src.initParent(this);
        PC_SOURCE_OUTPUT.initParent(this);
        JUMP_IF_FLUSH_OUTPUT.initParent(this);
        CONTROL_LINES_OUTPUT.initParent(this);
        JUMP_OUTPUT.initParent(this);

        Opcode_Internal.fill(0); // Default opcode to 0.
        // All other _Internal control signals are already default-initialized to 0 by their declarations.
    }
    ~Controller() override = default; // Default destructor.

    /**
     * @brief Simulates control logic for one cycle.
     * Reads the opcode and generates all necessary control signals for the datapath.
     * These signals are then combined into an 8-bit bundle for the ID/EX register,
     * while some (PCSource, Jump related) are output individually.
     * @param cycle The current simulation cycle number (for debugging).
     */
    void DriveUnit(int cycle = 0) override {
        // 1. Read the opcode input.
        Opcode_Src.read(Opcode_Internal);
        unsigned int opcode_val = Convert::ArrayToUInt<4>(Opcode_Internal);

        // 2. Reset all internal control signals to their default (inactive/NOP) state for this cycle.
        RegDst_Internal = { 0 }; ALUSrc_Internal = { 0 }; MemToReg_Internal = { 0 };
        RegWrite_Internal = { 0 }; MemRead_Internal = { 0 }; MemWrite_Internal = { 0 };
        PCSource_Internal = { 0 }; ALUOp_Internal = { 0, 0 }; Jump_IF_Flush_Internal = { 0 };
        Jump_Internal = { 0 };

        // 3. Decode the opcode and set appropriate control signals.
        //    Opcode values based on your MIPS subset:
        //    0:R-type, 1:addi, 2:lui, 3:ori, 4:sw, 5:lw, 6:bne, 7:j
        switch (opcode_val) {
        case 0: // R-type (e.g., add, sub, slt, sll, srl)
            RegDst_Internal = { 1 };
            RegWrite_Internal = { 1 };
            ALUOp_Internal = { 0, 1 }; // ALUOp = "10" (binary 2)
            break;
        case 1: // addi
            ALUSrc_Internal = { 1 };
            RegWrite_Internal = { 1 };
            ALUOp_Internal = { 0, 0 }; // ALUOp = "00" (binary 0)
            break;
        case 2: // lui (load upper immediate)
            ALUSrc_Internal = { 1 };    // ALU Input B is immediate.
            RegWrite_Internal = { 1 };  // Enable register write (to Rt).
            ALUOp_Internal = { 1, 1 };  // NEW: ALUOp = "11" (binary 3) for special immediate logic
            // Rs is implicitly $r0 (value 0), which will make ALU.InputA zero.
            break;
        case 3: // ori
            ALUSrc_Internal = { 1 };    // ALU Input B is immediate.
            RegWrite_Internal = { 1 };  // Enable register write (to Rt).
            ALUOp_Internal = { 1, 1 };  // NEW: ALUOp = "11" (binary 3) for special immediate logic
            break;
        case 4: // sw (store word)
            ALUSrc_Internal = { 1 };
            MemWrite_Internal = { 1 };
            ALUOp_Internal = { 0, 0 }; // ALUOp = "00"
            break;
        case 5: // lw (load word)
            ALUSrc_Internal = { 1 };
            MemToReg_Internal = { 1 };
            RegWrite_Internal = { 1 };
            MemRead_Internal = { 1 };
            ALUOp_Internal = { 0, 0 };  // ALUOp = "00"
            break;
        case 6: // bne (branch if not equal)
            PCSource_Internal = { 1 };
            ALUOp_Internal = { 1, 0 }; // ALUOp = "01" (binary 1)
            break;
        case 7: // j (jump)
            Jump_IF_Flush_Internal = { 1 };
            Jump_Internal = { 1 };
            break;
        default: // Unknown Opcode - treat as NOP.
            break;
        }

        // 4. Combine relevant control signals into an 8-bit bundle for ID/EX pipeline register.
        //    Layout (LSB first): [0:ALUSrc] [1-2:ALUOp] [3:RegDst] [4:MemWrite] [5:MemRead] [6:MemToReg] [7:RegWrite]
        Combined_Controls_Output_Internal[0] = ALUSrc_Internal[0];
        Combined_Controls_Output_Internal[1] = ALUOp_Internal[0];   // LSB of ALUOp
        Combined_Controls_Output_Internal[2] = ALUOp_Internal[1];   // MSB of ALUOp
        Combined_Controls_Output_Internal[3] = RegDst_Internal[0];
        Combined_Controls_Output_Internal[4] = MemWrite_Internal[0];
        Combined_Controls_Output_Internal[5] = MemRead_Internal[0];
        Combined_Controls_Output_Internal[6] = MemToReg_Internal[0];
        Combined_Controls_Output_Internal[7] = RegWrite_Internal[0];

        // Debug print for cycle 8.
        if (cycle == 8)
        {
            std::cout << component_name
                << " Opcode_Internal = " << Convert::ArrayToUInt(Opcode_Internal)
                << " ALUSrc_Internal = " << ALUSrc_Internal[0]
                << " ALUOp_Internal[0] = " << ALUOp_Internal[0]
                << " ALUOp_Internal[1] = " << ALUOp_Internal[1]
                << " RegDst_Internal[0] = " << RegDst_Internal[0]
                << " MemWrite_Internal[0] = " << MemWrite_Internal[0]
                << " MemRead_Internal[0] = " << MemRead_Internal[0]
                << " MemToReg_Internal[0] = " << MemToReg_Internal[0]
                << " RegWrite_Internal[0] = " << RegWrite_Internal[0]
                << " PCSource_Internal = " << PCSource_Internal[0]
                << " Jump_IF_Flush_Internal = " << Jump_IF_Flush_Internal[0]
                << " Jump_Internal = " << Jump_Internal[0] << std::endl;
        }
    }

    /**
     * @brief Writes the generated control signals to their respective output targets.
     */
    void WriteOutput() override {
        // Write individual control signals needed by components before ID/EX stage or for PC update.
        PC_SOURCE_OUTPUT.write(PCSource_Internal);         // To Branch AND Gate.
        JUMP_IF_FLUSH_OUTPUT.write(Jump_IF_Flush_Internal); // To Flush OR Gate (for PipelineRegisters).
        JUMP_OUTPUT.write(Jump_Internal);                   // To Jump Mux selector.
        // Write the combined 8-bit control signal bundle (for ID/EX register).
        CONTROL_LINES_OUTPUT.write(Combined_Controls_Output_Internal);
    }
};

/**
 * @brief Simulates a Sign Extender unit.
 *
 * Takes a 12-bit input (typically an immediate value from an I-type instruction)
 * and sign-extends it to a 24-bit value. The sign bit of the 12-bit input
 * (bit 11) is replicated to fill the upper 12 bits of the 24-bit output.
 */
class SignExtender : public DriveableUnit {
public:
    // --- State Variables ---
    /// Internal buffer for the 12-bit input value (LSB at index 0).
    std::array<unsigned int, 12> Input_Internal;
    /// Internal buffer for the 24-bit sign-extended output value (LSB at index 0).
    std::array<unsigned int, 24> Output_Internal;

    // --- Input Ports ---
    /// Input source for the 12-bit value to be sign-extended.
    /// This typically comes from bits 0-11 of the instruction in the IF/ID register.
    InputSliceSource<12> Input_Src{ "Input_Src" };

    // --- Output Ports ---
    /// Primary output target for the 24-bit sign-extended value (e.g., to ID/EX register).
    OutputSliceTarget<24> OUTPUT_1{ "OUTPUT_1" };
    /// Optional second output target (e.g., to a multiplier for branch offset calculation).
    OutputSliceTarget<24> OUTPUT_2{ "OUTPUT_2" };

    /**
     * @brief Constructor for the SignExtender component.
     * Initializes port names and internal buffers.
     * @param name The name of this SignExtender instance.
     */
    SignExtender(const std::string& name = "SignExtender") : DriveableUnit(name) {
        // Initialize input and output ports with parent component information.
        Input_Src.initParent(this);
        OUTPUT_1.initParent(this);
        OUTPUT_2.initParent(this);

        // Initialize internal state buffers to zero.
        Input_Internal.fill(0);
        Output_Internal.fill(0);
    }
    ~SignExtender() override = default; // Default destructor.

    /**
     * @brief Performs the 12-bit to 24-bit sign extension.
     * The input array is LSB-first (index 0 is bit 0). The MSB of the 12-bit
     * input is at index 11. This sign bit is replicated to bits 12 through 23
     * of the 24-bit output.
     * @param cycle The current simulation cycle number (for debugging).
     */
    void DriveUnit(int cycle = 0) override {
        // 1. Read the 12-bit input into the internal buffer.
        Input_Src.read(Input_Internal);

        // 2. Get the sign bit (MSB of the 12-bit input, which is at index 11).
        unsigned int signBit = Input_Internal[11] & 1; // Ensure it's 0 or 1.

        // 3. Copy the original 12 bits to the lower 12 bits of the 24-bit output.
        //    Output_Internal[0] = Input_Internal[0], ..., Output_Internal[11] = Input_Internal[11].
        for (size_t i = 0; i < 12; ++i) {
            Output_Internal[i] = Input_Internal[i] & 1; // Ensure only 0 or 1 is copied.
        }

        // 4. Fill the upper 12 bits (indices 12-23) of the output with the sign bit.
        for (size_t i = 12; i < 24; ++i) {
            Output_Internal[i] = signBit;
        }

        // Debug print for cycle 8.
        if (cycle == 8)
        {
            std::cout << component_name << " Input_Internal = " << Convert::ArrayToUInt(Input_Internal)
                << " Output_Internal = " << Convert::ArrayToUInt(Output_Internal) << std::endl;
        }
    }

    /**
     * @brief Writes the 24-bit sign-extended result to its configured output targets.
     */
    void WriteOutput() override {
        // Write the result to both output ports if they are connected.
        OUTPUT_1.write(Output_Internal);
        OUTPUT_2.write(Output_Internal); // Often, the same extended value is needed by multiple components.
    }
};

/**
 * @brief Component that performs multiplication by 3.
 *
 * This unit is designed to handle two independent multiplication paths:
 * 1. A 24-bit input multiplied by 3, producing a 24-bit output.
 * 2. A 20-bit input multiplied by 3, producing a 22-bit output.
 *
 * The multiplication by 3 is implemented as `(input << 1) + input`, which is
 * equivalent to `(input * 2) + input = input * 3`.
 */
class Multiply3 : public DriveableUnit {
public:
    // --- State Variables ---
    // For the 24-bit input/output path:
    std::array<unsigned int, 24> Input24_Internal;   ///< Internal buffer for the 24-bit input.
    std::array<unsigned int, 24> Output24_Internal;  ///< Internal buffer for the 24-bit result (Input24_Internal * 3).

    // For the 20-bit input to 22-bit output path:
    std::array<unsigned int, 20> Input20_Internal;   ///< Internal buffer for the 20-bit input.
    std::array<unsigned int, 22> Output22_Internal;  ///< Internal buffer for the 22-bit result (Input20_Internal * 3).
    // Output is 22 bits because 2^20 * 3 < 2^22.

// --- Input Ports ---
/// Input source for the 24-bit value to be multiplied by 3.
    InputSliceSource<24> Input24_Src{ "Input24_Src" };
    /// Input source for the 20-bit value to be multiplied by 3.
    InputSliceSource<20> Input20_Src{ "Input20_Src" };

    // --- Output Ports ---
    /// Output target for the 24-bit result (Input24 * 3).
    OutputSliceTarget<24> OUTPUT24{ "OUTPUT24" };
    /// Output target for the 22-bit result (Input20 * 3).
    OutputSliceTarget<22> OUTPUT22{ "OUTPUT22" };

    /**
     * @brief Constructor for the Multiply3 component.
     * Initializes port names and internal buffers.
     * @param name The name of this Multiply3 instance.
     */
    Multiply3(const std::string& name = "Multiply3") : DriveableUnit(name) {
        // Initialize all input and output ports with parent component information.
        Input24_Src.initParent(this);
        Input20_Src.initParent(this);
        OUTPUT24.initParent(this);
        OUTPUT22.initParent(this);

        // Initialize internal state buffers to all zeros.
        Input24_Internal.fill(0);
        Output24_Internal.fill(0);
        Input20_Internal.fill(0);
        Output22_Internal.fill(0);
    }
    ~Multiply3() override = default; // Default destructor.

    /**
     * @brief Simulates the multiplication by 3 operations for one cycle.
     * Reads inputs for both the 24-bit path and the 20-bit path (if validly connected).
     * Calculates `input * 3` for each path and stores results in internal output buffers.
     * The multiplication is implemented as `(input << 1) + input`.
     * @param cycle The current simulation cycle number (for debugging).
     */
    void DriveUnit(int cycle = 0) override {
        // 1. Read inputs if their respective sources are validly configured.
        if (Input24_Src.isValid()) { // Check if the 24-bit input source is connected.
            Input24_Src.read(Input24_Internal);
        }
        if (Input20_Src.isValid()) { // Check if the 20-bit input source is connected.
            Input20_Src.read(Input20_Internal);
        }

        // 2. Perform 24-bit input * 3 operation.
        unsigned int value24_in = Convert::ArrayToUInt<24>(Input24_Internal); // Convert 24-bit array to integer.
        unsigned int value24_shifted = value24_in << 1;                       // Calculate value * 2.
        unsigned int value24_result = value24_shifted + value24_in;           // Calculate (value * 2) + value.
        // Convert the integer result back to a 24-bit array.
        // If value24_result exceeds 24 bits, ToArray will take the 24 LSBs.
        Output24_Internal = Convert::ToArray(value24_result);

        // 3. Perform 20-bit input * 3 operation (result fits in 22 bits).
        unsigned int value20_in = Convert::ArrayToUInt<20>(Input20_Internal); // Convert 20-bit array to integer.
        unsigned int value20_shifted = value20_in << 1;                       // Calculate value * 2.
        unsigned int value20_result = value20_shifted + value20_in;           // Calculate (value * 2) + value.
        // Convert the integer result to a 22-bit array.
        // Max 20-bit value: (2^20 - 1). Max result: 3*(2^20 - 1) = 3*2^20 - 3.
        // 2^21 = 2*2^20. 2^22 = 4*2^20. So, 3*2^20 fits within 22 bits.
        Output22_Internal = Convert::ToArray22Bit(value20_result);

        // Debug print for cycle 8.
        if (cycle == 8)
        {
            if (OUTPUT22.isValid()) // Check if the output port is actually connected to something.
            {
                std::cout << component_name << " Input20_Internal = " << Convert::ArrayToUInt(Input20_Internal)
                    << " Output22_Internal = " << Convert::ArrayToUInt(Output22_Internal) << std::endl;
            }
            if (OUTPUT24.isValid()) // Check if the output port is actually connected.
            {
                std::cout << component_name << " Input24_Internal = " << Convert::ArrayToUInt(Input24_Internal)
                    << " Output24_Internal = " << Convert::ArrayToUInt(Output24_Internal) << std::endl;
            }
        }
    }

    /**
     * @brief Writes the calculated results (Output24_Internal, Output22_Internal)
     * to their respective configured output targets.
     * Writes only occur if the output ports are validly connected.
     */
    void WriteOutput() override {
        // Write the 22-bit result if its output target is valid.
        if (OUTPUT22.isValid()) {
            OUTPUT22.write(Output22_Internal);
        }
        // Write the 24-bit result if its output target is valid.
        if (OUTPUT24.isValid()) {
            OUTPUT24.write(Output24_Internal);
        }
    }
};

/**
 * @brief Simulates a 2-input, 1-bit AND gate.
 *
 * Performs a logical AND operation on two single-bit inputs.
 * Has an optional second output.
 */
class AndGater : public DriveableUnit {
public:
    // --- State Variables ---
    /// Internal buffer for the first 1-bit input.
    std::array<unsigned int, 1> InputA_Internal;
    /// Internal buffer for the second 1-bit input.
    std::array<unsigned int, 1> InputB_Internal;
    /// Internal buffer for the 1-bit result of (InputA AND InputB).
    std::array<unsigned int, 1> OUTPUT_Internal;

    // --- Input Ports ---
    /// Input source for the first 1-bit operand.
    InputSliceSource<1> InputA_Src{ "InputA_Src" };
    /// Input source for the second 1-bit operand.
    InputSliceSource<1> InputB_Src{ "InputB_Src" };

    // --- Output Ports ---
    /// Primary output target for the 1-bit AND result.
    OutputSliceTarget<1> OUTPUT{ "OUTPUT" };
    /// Optional second output target for the AND result (e.g., for flushing logic).
    OutputSliceTarget<1> OUTPUT_2{ "OUTPUT_2" };

    /**
     * @brief Constructor for the AndGater component.
     * Initializes port names and internal buffers.
     * @param name The name of this AndGater instance.
     */
    AndGater(const std::string& name = "AndGater") : DriveableUnit(name) {
        // Initialize input and output ports with parent component information.
        InputA_Src.initParent(this);
        InputB_Src.initParent(this);
        OUTPUT.initParent(this);
        // OUTPUT_2 is also initialized implicitly by its constructor.

        // Initialize internal state buffers to 0.
        InputA_Internal[0] = 0;
        InputB_Internal[0] = 0;
        OUTPUT_Internal[0] = 0;
    }
    ~AndGater() override = default; // Default destructor.

    /**
     * @brief Simulates the AND gate logic for one cycle.
     * Reads the two 1-bit inputs and computes their logical AND.
     * The specific logic implemented here is for a BNE (Branch if Not Equal) instruction:
     * Output = BranchSignal AND (NOT ZeroFlag).
     * Where InputA is BranchSignal and InputB is ZeroFlag (1 if equal, 0 if not equal).
     * @param cycle The current simulation cycle number (for debugging).
     */
    void DriveUnit(int cycle = 0) override {
        // 1. Read inputs into internal buffers.
        InputA_Src.read(InputA_Internal); // Typically the 'Branch' control signal from Control Unit.
        InputB_Src.read(InputB_Internal); // Typically the 'Equal' or 'Zero' flag (1 if equal/zero).

        // 2. Perform logical AND operation.
        // The current logic is tailored for a BNE (Branch if Not Equal) instruction.
        // For BNE, branch is taken if: BranchSignal (InputA) is 1 AND inputs are NOT equal (InputB is 0).
        // So, we need to invert InputB if it represents an "Equal" flag.
        unsigned int inputA_bit = InputA_Internal[0] & 1;
        unsigned int inputB_bit_original = InputB_Internal[0] & 1;

        // condition_met_for_bne: result of (NOT InputB) if InputB means "Equal"
        // If InputB means "Equal", then (InputB ^ 1) means "Not Equal".
        unsigned int condition_for_branch = inputB_bit_original ^ 1; // Invert InputB (if 1 means equal, 0 means not equal)

        // Output is 1 if (BranchSignal is 1) AND (condition_for_branch is 1)
        OUTPUT_Internal[0] = inputA_bit & condition_for_branch;

        // If this AND gate were for BEQ (Branch if Equal):
        // Output = BranchSignal AND EqualFlag
        // OUTPUT_Internal[0] = (InputA_Internal[0] & 1) & (InputB_Internal[0] & 1);

        // Debug print for cycle 8.
        if (cycle == 8)
        {
            std::cout << component_name << " OUTPUT_INTERNAL = " << OUTPUT_Internal[0]
                << " InputA_Internal (BranchSignal) = " << Convert::ArrayToUInt(InputA_Internal)
                << " InputB_Internal (EqualFlag) = " << Convert::ArrayToUInt(InputB_Internal) << std::endl;
        }
    }

    /**
     * @brief Writes the 1-bit AND result to its configured output target(s).
     */
    void WriteOutput() override {
        OUTPUT.write(OUTPUT_Internal);
        // If OUTPUT_2 is connected (e.g., to a flush logic OR gate), write to it.
        if (OUTPUT_2.isValid()) {
            OUTPUT_2.write(OUTPUT_Internal);
        }
    }
};

/**
 * @brief Simulates a 2-input, 1-bit OR gate.
 *
 * Performs a logical OR operation on two single-bit inputs.
 */
class OrGater : public DriveableUnit {
public:
    // --- State Variables ---
    /// Internal buffer for the first 1-bit input.
    std::array<unsigned int, 1> InputA_Internal;
    /// Internal buffer for the second 1-bit input.
    std::array<unsigned int, 1> InputB_Internal;
    /// Internal buffer for the 1-bit result of (InputA OR InputB).
    std::array<unsigned int, 1> OUTPUT_Internal;

    // --- Input Ports ---
    /// Input source for the first 1-bit operand.
    InputSliceSource<1> InputA_Src{ "InputA_Src" };
    /// Input source for the second 1-bit operand.
    InputSliceSource<1> InputB_Src{ "InputB_Src" };

    // --- Output Ports ---
    /// Output target for the 1-bit OR result.
    OutputSliceTarget<1> OUTPUT{ "OUTPUT" };

    /**
     * @brief Constructor for the OrGater component.
     * Initializes port names and internal buffers.
     * @param name The name of this OrGater instance.
     */
    OrGater(const std::string& name = "OrGater") : DriveableUnit(name) {
        // Initialize input and output ports with parent component information.
        InputA_Src.initParent(this);
        InputB_Src.initParent(this);
        OUTPUT.initParent(this);

        // Initialize internal state buffers to 0.
        InputA_Internal[0] = 0;
        InputB_Internal[0] = 0;
        OUTPUT_Internal[0] = 0;
    }
    ~OrGater() override = default; // Default destructor.

    /**
     * @brief Simulates the OR gate logic for one cycle.
     * Reads the two 1-bit inputs and computes their logical OR.
     * @param cycle The current simulation cycle number (for debugging).
     */
    void DriveUnit(int cycle = 0) override {
        // 1. Read inputs into internal buffers.
        InputA_Src.read(InputA_Internal);
        InputB_Src.read(InputB_Internal);

        // 2. Perform logical OR operation.
        // Ensure inputs are treated as single bits (0 or 1).
        unsigned int inputA_bit = InputA_Internal[0] & 1;
        unsigned int inputB_bit = InputB_Internal[0] & 1;

        // Bitwise OR on single bits is equivalent to logical OR.
        OUTPUT_Internal[0] = (inputA_bit | inputB_bit);

        // Debug print for cycle 8.
        if (cycle == 8)
        {
            std::cout << component_name << " OUTPUT_Internal = " << OUTPUT_Internal[0]
                << " InputA_Internal (e.g., From Control Unit JumpFlush) = " << inputA_bit
                << " InputB_Internal (e.g., From Branch Taken) = " << inputB_bit << std::endl;
        }
    }

    /**
     * @brief Writes the 1-bit OR result to its configured output target.
     */
    void WriteOutput() override {
        OUTPUT.write(OUTPUT_Internal);
    }
};

/**
 * @brief A utility component that directly passes through specific data fields
 *        from one pipeline stage's output register to the next stage's input buffer.
 *
 * This component centralizes various direct connections of fields (like register numbers,
 * control signals) that don't undergo transformation but are simply carried along
 * the pipeline. It reads from slices of one pipeline register (e.g., IF/ID) and
 * writes to slices of the _Next_Input buffer of the subsequent pipeline register
 * (e.g., ID/EX_Next_Input).
 *
 */
class PassThrough : public DriveableUnit {
public:
    // --- State: Internal buffers for each pass-through connection ---
    // These buffers temporarily hold the data being passed.

    // Fields passed from IF/ID to ID/EX_Next_Input
    std::array<unsigned int, 4> IFID_TO_IDEX_RS_Internal;    ///< Rs register number.
    std::array<unsigned int, 4> IFID_TO_IDEX_RT_Internal;    ///< Rt register number.
    std::array<unsigned int, 4> IFID_TO_IDEX_RD_Internal;    ///< Rd register number.

    // Fields passed from ID/EX to EX/MEM_Next_Input
    std::array<unsigned int, 2> IDEX_TO_EXMEM_WB_Internal;   ///< WB Controls (RegWrite, MemToReg).
    std::array<unsigned int, 2> IDEX_TO_EXMEM_M_Internal;    ///< M Controls (MemRead, MemWrite).

    // Fields passed from EX/MEM to MEM/WB_Next_Input
    std::array<unsigned int, 2> EXMEM_TO_MEMWB_WB_Internal;  ///< WB Controls (RegWrite, MemToReg).
    std::array<unsigned int, 4> EXMEM_TO_MEMWB_WRITE_REGISTER_Internal; ///< Write Register number.
    std::array<unsigned int, 24> EXMEM_TO_MEMWB_ALU_RESULT_Internal;   ///< ALU Result.

    // --- Input Ports (Sources are slices from pipeline registers) ---
    InputSliceSource<4> IFID_TO_IDEX_RS_Src{ "IFID_TO_IDEX_RS_Src" }; ///< From IF/ID.Rs field.
    InputSliceSource<4> IFID_TO_IDEX_RT_Src{ "IFID_TO_IDEX_RT_Src" }; ///< From IF/ID.Rt field.
    InputSliceSource<4> IFID_TO_IDEX_RD_Src{ "IFID_TO_IDEX_RD_Src" }; ///< From IF/ID.Rd field.

    InputSliceSource<2> IDEX_TO_EXMEM_WB_Src{ "IDEX_TO_EXMEM_WB_Src" }; ///< From ID/EX.WBControls field.
    InputSliceSource<2> IDEX_TO_EXMEM_M_Src{ "IDEX_TO_EXMEM_M_Src" };   ///< From ID/EX.MControls field.

    InputSliceSource<2> EXMEM_TO_MEMWB_WB_Src{ "EXMEM_TO_MEMWB_WB_Src" }; ///< From EX/MEM.WBControls field.
    InputSliceSource<4> EXMEM_TO_MEMWB_WRITE_REGISTER_Src{ "EXMEM_TO_MEMWB_WRITE_REGISTER_Src" }; ///< From EX/MEM.WriteRegister# field.
    InputSliceSource<24> EXMEM_TO_MEMWB_ALU_RESULT_Src{ "EXMEM_TO_MEMWB_ALU_RESULT_Src" }; ///< From EX/MEM.ALUResult field.

    // --- Output Ports (Targets are slices in _Next_Input buffers of pipeline registers) ---
    OutputSliceTarget<4> IFID_TO_IDEX_RS_OUTPUT{ "IFID_TO_IDEX_RS_OUTPUT" }; ///< To ID/EX_Next_Input.Rs# field.
    OutputSliceTarget<4> IFID_TO_IDEX_RT_OUTPUT{ "IFID_TO_IDEX_RT_OUTPUT" }; ///< To ID/EX_Next_Input.Rt# field.
    OutputSliceTarget<4> IFID_TO_IDEX_RD_OUTPUT{ "IFID_TO_IDEX_RD_OUTPUT" }; ///< To ID/EX_Next_Input.Rd# field.

    OutputSliceTarget<2> IDEX_TO_EXMEM_WB_OUTPUT{ "IDEX_TO_EXMEM_WB_OUTPUT" }; ///< To EX/MEM_Next_Input.WBControls field.
    OutputSliceTarget<2> IDEX_TO_EXMEM_M_OUTPUT{ "IDEX_TO_EXMEM_M_OUTPUT" };   ///< To EX/MEM_Next_Input.MControls field.

    OutputSliceTarget<2> EXMEM_TO_MEMWB_WB_OUTPUT{ "EXMEM_TO_MEMWB_WB_OUTPUT" }; ///< To MEM/WB_Next_Input.WBControls field.
    OutputSliceTarget<4> EXMEM_TO_MEMWB_WRITE_REGISTER_OUTPUT{ "EXMEM_TO_MEMWB_WRITE_REGISTER_OUTPUT" }; ///< To MEM/WB_Next_Input.WriteRegister# field.
    OutputSliceTarget<24> EXMEM_TO_MEMWB_ALU_RESULT_OUTPUT{ "EXMEM_TO_MEMWB_ALU_RESULT_OUTPUT" }; ///< To MEM/WB_Next_Input.ALUResult field.

    /**
     * @brief Constructor for the PassThrough component.
     * Initializes all input and output ports. Internal buffers are default-initialized (to zero).
     * @param name The name of this PassThrough instance.
     */
    PassThrough(const std::string& name = "PassThrough") : DriveableUnit(name) {
        // Initialize all input ports with parent info.
        IFID_TO_IDEX_RS_Src.initParent(this);
        IFID_TO_IDEX_RT_Src.initParent(this);
        IFID_TO_IDEX_RD_Src.initParent(this);
        IDEX_TO_EXMEM_WB_Src.initParent(this);
        IDEX_TO_EXMEM_M_Src.initParent(this);
        EXMEM_TO_MEMWB_WB_Src.initParent(this);
        EXMEM_TO_MEMWB_WRITE_REGISTER_Src.initParent(this);
        EXMEM_TO_MEMWB_ALU_RESULT_Src.initParent(this);

        // Initialize all output ports with parent info.
        IFID_TO_IDEX_RS_OUTPUT.initParent(this);
        IFID_TO_IDEX_RT_OUTPUT.initParent(this);
        IFID_TO_IDEX_RD_OUTPUT.initParent(this);
        IDEX_TO_EXMEM_WB_OUTPUT.initParent(this);
        IDEX_TO_EXMEM_M_OUTPUT.initParent(this);
        EXMEM_TO_MEMWB_WB_OUTPUT.initParent(this);
        EXMEM_TO_MEMWB_WRITE_REGISTER_OUTPUT.initParent(this);
        EXMEM_TO_MEMWB_ALU_RESULT_OUTPUT.initParent(this);

        // Internal buffers are default-initialized to all zeros by their std::array constructors.
    }
    ~PassThrough() override = default; // Default destructor.

    /**
     * @brief Simulates the pass-through action for one cycle.
     * Reads data from all configured input sources into their corresponding internal buffers.
     * @param cycle The current simulation cycle number (for debugging).
     */
    void DriveUnit(int cycle = 0) override {
        // Read data from IF/ID register slices.
        IFID_TO_IDEX_RS_Src.read(IFID_TO_IDEX_RS_Internal);
        IFID_TO_IDEX_RT_Src.read(IFID_TO_IDEX_RT_Internal);
        IFID_TO_IDEX_RD_Src.read(IFID_TO_IDEX_RD_Internal);

        // Read data from ID/EX register slices.
        IDEX_TO_EXMEM_WB_Src.read(IDEX_TO_EXMEM_WB_Internal);
        IDEX_TO_EXMEM_M_Src.read(IDEX_TO_EXMEM_M_Internal);

        // Read data from EX/MEM register slices.
        EXMEM_TO_MEMWB_WB_Src.read(EXMEM_TO_MEMWB_WB_Internal);
        EXMEM_TO_MEMWB_WRITE_REGISTER_Src.read(EXMEM_TO_MEMWB_WRITE_REGISTER_Internal);
        EXMEM_TO_MEMWB_ALU_RESULT_Src.read(EXMEM_TO_MEMWB_ALU_RESULT_Internal);
    }

    /**
     * @brief Writes data from internal buffers to their corresponding output targets.
     * This effectively transfers the read data to the _Next_Input buffers of the
     * subsequent pipeline registers.
     */
    void WriteOutput() override {
        // Write to ID/EX_Next_Input slices.
        IFID_TO_IDEX_RS_OUTPUT.write(IFID_TO_IDEX_RS_Internal);
        IFID_TO_IDEX_RT_OUTPUT.write(IFID_TO_IDEX_RT_Internal);
        IFID_TO_IDEX_RD_OUTPUT.write(IFID_TO_IDEX_RD_Internal);

        // Write to EX/MEM_Next_Input slices.
        IDEX_TO_EXMEM_WB_OUTPUT.write(IDEX_TO_EXMEM_WB_Internal);
        IDEX_TO_EXMEM_M_OUTPUT.write(IDEX_TO_EXMEM_M_Internal);

        // Write to MEM/WB_Next_Input slices.
        EXMEM_TO_MEMWB_WB_OUTPUT.write(EXMEM_TO_MEMWB_WB_Internal);
        EXMEM_TO_MEMWB_WRITE_REGISTER_OUTPUT.write(EXMEM_TO_MEMWB_WRITE_REGISTER_Internal);
        EXMEM_TO_MEMWB_ALU_RESULT_OUTPUT.write(EXMEM_TO_MEMWB_ALU_RESULT_Internal);
    }
};

/**
 * @brief Component responsible for constructing the final jump target address.
 *
 * For a MIPS J-type instruction, the target address is formed by:
 * - Taking the 2 most significant bits from the current PC + increment_value (e.g., PC+3).
 * - Taking the 20-bit address field from the jump instruction.
 * - Multiplying this 20-bit field by the word size (3 bytes in this architecture).
 * - Concatenating these parts: (PC+inc)_msb2 | (instr_addr_field * 3)_lsb22.
 * The result is a 24-bit jump target address.
 */
class JumpAddressConcatenator : public DriveableUnit {
public:
    // --- State Variables ---
    /// Internal buffer for the 22-bit scaled address field from the jump instruction (instr_addr_field * 3).
    std::array<unsigned int, 22> ShiftedAddress_Internal;
    /// Internal buffer for the 2 most significant bits of PC + increment_value.
    std::array<unsigned int, 2> PC_MSB2_Internal;
    /// Internal buffer for the final 24-bit concatenated jump target address.
    std::array<unsigned int, 24> OUTPUT_Internal;

    // --- Input Ports ---
    /// Input source for the 22-bit scaled address from Multiply3 component.
    InputSliceSource<22> ShiftedAddress_Src{ "ShiftedAddress_Src" };
    /// Input source for the 2 MSBs of PC+increment (e.g., from PCIncrementAdder).
    InputSliceSource<2> PC_MSB2_Src{ "PC_MSB2_Src" };

    // --- Output Ports ---
    /// Output target for the calculated 24-bit jump target address (to Jump Mux).
    OutputSliceTarget<24> JumpTargetAddress_OUTPUT{ "JumpTargetAddress_OUTPUT" };

    /**
     * @brief Constructor for the JumpAddressConcatenator component.
     * Initializes port names and internal buffers.
     * @param name The name of this JumpAddressConcatenator instance.
     */
    JumpAddressConcatenator(const std::string& name = "JumpAddrConcat") : DriveableUnit(name) {
        // Initialize input and output ports with parent component information.
        ShiftedAddress_Src.initParent(this);
        PC_MSB2_Src.initParent(this);
        JumpTargetAddress_OUTPUT.initParent(this);

        // Initialize internal state buffers to zero.
        ShiftedAddress_Internal.fill(0);
        PC_MSB2_Internal.fill(0);
        OUTPUT_Internal.fill(0);
    }
    ~JumpAddressConcatenator() override = default; // Default destructor.

    /**
     * @brief Simulates the concatenation logic for one cycle.
     * Reads the shifted instruction address field and the PC's MSBs,
     * then concatenates them to form the 24-bit jump target address.
     * The output array is LSB-first:
     * OUTPUT_Internal[0-21]   <- ShiftedAddress_Internal[0-21]
     * OUTPUT_Internal[22]     <- PC_MSB2_Internal[0] (PC_MSB2 LSB, which is overall bit 22)
     * OUTPUT_Internal[23]     <- PC_MSB2_Internal[1] (PC_MSB2 MSB, which is overall bit 23)
     * @param cycle The current simulation cycle number (for debugging).
     */
    void DriveUnit(int cycle = 0) override {
        // 1. Read inputs into internal buffers.
        ShiftedAddress_Src.read(ShiftedAddress_Internal); // 22 LSBs of target
        PC_MSB2_Src.read(PC_MSB2_Internal);             // 2 MSBs of target (from PC region)

        // 2. Perform concatenation to form the 24-bit jump address.
        // Copy the lower 22 bits from the shifted address.
        for (size_t i = 0; i < 22; ++i) {
            OUTPUT_Internal[i] = ShiftedAddress_Internal[i];
        }
        // Copy the upper 2 bits from PC_MSB2_Internal.
        // Ensure array is large enough (always true for 24-bit output).
        if (24 >= 2) {
            OUTPUT_Internal[22] = PC_MSB2_Internal[0]; // PC_MSB2[0] is bit 22 of the final address.
            OUTPUT_Internal[23] = PC_MSB2_Internal[1]; // PC_MSB2[1] is bit 23 (MSB) of the final address.
        }

        // Debug print for cycle 8.
        if (cycle == 8)
        {
            std::cout << component_name
                << " ShiftedAddress_Internal (instr_target*3) = " << Convert::ArrayToUInt(ShiftedAddress_Internal)
                << " PC_MSB2_Internal (from PC+inc) = " << Convert::ArrayToUInt(PC_MSB2_Internal)
                << " OUTPUT_Internal (Final Jump Target) = " << Convert::ArrayToUInt(OUTPUT_Internal) << std::endl;
        }
    }

    /**
     * @brief Writes the calculated 24-bit jump target address to its output target.
     */
    void WriteOutput() override {
        JumpTargetAddress_OUTPUT.write(OUTPUT_Internal);
    }
};

/**
 * @brief Provides static methods for encoding assembly instructions into 24-bit
 *        machine code and decoding machine code back into assembly strings.
 *
 * This class supports a subset of MIPS-like instructions tailored for the
 * 24-bit architecture being simulated. It includes mappings for registers,
 * opcodes, funct codes, and instruction formats (R, I, J).
 * All methods are static, so no instantiation of InstructionDecoder is needed.
 */
class InstructionDecoder {
private:
    // --- Data Structures for Encoding/Decoding ---

    /// Maps MIPS register mnemonics (e.g., "$v0") to their 4-bit numerical representation.
    inline static const std::map<std::string, unsigned int> regNameToNum = {
        {"$r0", 0}, {"$zero", 0}, // Register 0 is $zero or $r0
        {"$v0", 1},               // Register 1
        {"$v1", 2},               // Register 2
        {"$v2", 3},               // Register 3
        {"$v3", 4},               // Register 4
        {"$t0", 5},               // Register 5
        {"$a0", 6},               // Register 6
        {"$a1", 7},               // Register 7
        {"$at", 8}                // Register 8 ($at - assembler temporary, often reg 1)
        // Note: Your mapping uses 8 for $at. Standard MIPS $at is usually $1.
        // This custom mapping is specific to this processor design.
    };

    /// Maps 4-bit numerical register representation back to their mnemonics.
    inline static const std::array<const char*, 9> regNumToName = {
        "$r0", "$v0", "$v1", "$v2", "$v3", "$t0", "$a0", "$a1", "$at"
    };

    /// @brief Structure to hold information about each instruction's encoding.
    struct InstInfo {
        unsigned int opcode;    ///< The 4-bit opcode value.
        unsigned int funct;     ///< The 4-bit funct code value (used only for R-type instructions).
        char format;            ///< Instruction format: 'R', 'I', or 'J'.
        int num_operands;       ///< Expected number of operands in assembly syntax (for parsing validation).
    };

    /// Maps instruction mnemonics to their InstInfo structure.
    /// Defines the opcode, funct (if R-type), format, and operand count for each supported instruction.
    inline static const std::map<std::string, InstInfo> mnemonicToInfo = {
        // R-Type Instructions (Opcode 0)
        {"add",     {0, 1, 'R', 3}}, // add rd, rs, rt
        {"sub",     {0, 2, 'R', 3}}, // sub rd, rs, rt
        {"multlow", {0, 3, 'R', 3}}, // multlow rd, rs, rt (lower 24 bits of product)
        {"and",     {0, 4, 'R', 3}}, // and rd, rs, rt
        {"or",      {0, 5, 'R', 3}}, // or rd, rs, rt
        {"xor",     {0, 6, 'R', 3}}, // xor rd, rs, rt
        {"slt",     {0, 7, 'R', 3}}, // slt rd, rs, rt
        {"sll",     {0, 8, 'R', 3}}, // sll rd, rt, shamt (Rs field is not used by ALU)
        {"srl",     {0, 9, 'R', 3}}, // srl rd, rt, shamt (Rs field is not used by ALU)
        {"nop",     {0, 8, 'R', 0}}, // NOP is encoded as sll $r0, $r0, 0 (Op=0, Rs=0, Rt=0, Rd=0, Shamt=0, Funct=8)

        // I-Type Instructions
        {"addi",    {1, 0, 'I', 3}}, // addi rt, rs, immediate
        {"lui",     {2, 0, 'I', 2}}, // lui rt, immediate (Rs field is 0)
        {"ori",     {3, 0, 'I', 3}}, // ori rt, rs, immediate
        {"sw",      {4, 0, 'I', 3}}, // sw rt, offset(rs)
        {"lw",      {5, 0, 'I', 3}}, // lw rt, offset(rs)
        {"bne",     {6, 0, 'I', 3}}, // bne rs, rt, offset

        // J-Type Instructions
        {"j",       {7, 0, 'J', 1}}  // j target_address
    };

    // --- Private Helper Functions for Encoding/Decoding ---

    /**
     * @brief Gets the register mnemonic string for a given numerical index.
     * @param index The 4-bit numerical representation of the register.
     * @return The string mnemonic (e.g., "$v0") or "$?" if index is invalid.
     */
    static std::string getRegName(unsigned int index) {
        if (index < regNumToName.size()) {
            return regNumToName[index];
        }
        return "$?"; // Fallback for unknown register index.
    }

    /**
     * @brief Gets the 4-bit numerical representation for a given register mnemonic string.
     * @param name The register mnemonic string (e.g., "$v0").
     * @param num Output parameter where the numerical value will be stored.
     * @return True if the name was found and num is set, false otherwise.
     */
    static bool getRegNum(const std::string& name, unsigned int& num) {
        auto it = regNameToNum.find(name);
        if (it != regNameToNum.end()) {
            num = it->second;
            return true;
        }
        return false; // Register name not found.
    }

    /**
     * @brief Formats a 12-bit immediate value (from an array) as a signed decimal string.
     * @param immArr The 12-bit array (LSB at index 0) representing the immediate.
     * @return A string of the signed decimal value.
     */
    static std::string formatSignedImm12(const std::array<unsigned int, 12>& immArr) {
        // Convert array to signed int (handles two's complement).
        int signedVal = Convert::ArrayToInt<12>(immArr);
        return std::to_string(signedVal);
    }

    /**
     * @brief Formats a 12-bit immediate value (from an array) as an unsigned hexadecimal string.
     * @param immArr The 12-bit array (LSB at index 0).
     * @return A string of the unsigned hex value (e.g., "0xfff").
     */
    static std::string formatUnsignedImm12Hex(const std::array<unsigned int, 12>& immArr) {
        unsigned int val = Convert::ArrayToUInt<12>(immArr);
        std::stringstream ss;
        ss << "0x" << std::hex << val; // Format as hexadecimal with "0x" prefix.
        return ss.str();
    }

    /**
     * @brief Formats a 20-bit jump target address (from an array) as an unsigned hexadecimal string.
     * @param immArr The 20-bit array (LSB at index 0).
     * @return A string of the unsigned hex value.
     */
    static std::string formatUnsignedImm20Hex(const std::array<unsigned int, 20>& immArr) {
        unsigned int val = Convert::ArrayToUInt<20>(immArr);
        std::stringstream ss;
        ss << "0x" << std::hex << val;
        return ss.str();
    }

    /**
     * @brief Parses an immediate value string (decimal or hexadecimal) into an integer.
     * Handles "0x" prefix and 'h' suffix for hexadecimal numbers.
     * @param s The string representation of the immediate value.
     * @param value Output parameter for the parsed integer value.
     * @param bits The number of bits the immediate value should fit into (for range checking).
     * @return True if parsing was successful and value is within range, false otherwise.
     */
    static bool parseImmediate(const std::string& s, int& value, int bits) {
        long long temp_val; // Use long long to avoid overflow during parsing before range check.
        std::string str_to_parse = s;
        int base = 10; // Default to decimal.

        // Check for hexadecimal prefixes/suffixes.
        if (s.size() > 2 && (s.substr(0, 2) == "0x" || s.substr(0, 2) == "0X")) {
            base = 16;
            str_to_parse = s.substr(2); // Remove "0x" prefix.
        }
        else if (s.size() > 1 && (s.back() == 'h' || s.back() == 'H')) {
            base = 16;
            str_to_parse = s.substr(0, s.size() - 1); // Remove 'h' suffix.
        }

        // Attempt to parse the string using std::from_chars (C++17).
        auto [ptr, ec] = std::from_chars(str_to_parse.data(), str_to_parse.data() + str_to_parse.size(), temp_val, base);

        // Check if parsing was successful (no error and all characters consumed).
        if (ec == std::errc() && ptr == str_to_parse.data() + str_to_parse.size()) {
            // Determine min/max valid range based on the number of bits.
            long long min_val, max_val;
            if (bits <= 12) { // For 12-bit immediates (e.g., addi, lw offset), can be signed.
                // Also allow full unsigned range for logicals like 'ori' or 'lui' upper bits.
                min_val = -(1LL << (bits - 1));      // Smallest signed N-bit number.
                max_val = (1LL << (bits - 1)) - 1LL; // Largest signed N-bit number.
                long long max_unsigned_val = (1LL << bits) - 1LL; // Largest unsigned N-bit number.

                // If value is positive and fits unsigned, or fits signed range.
                if ((temp_val >= 0 && temp_val <= max_unsigned_val) || (temp_val >= min_val && temp_val <= max_val)) {
                    value = static_cast<int>(temp_val);
                    return true;
                }
            }
            else { // For larger fields like 20-bit jump address, assume unsigned.
                min_val = 0;
                max_val = (1LL << bits) - 1LL;
                if (temp_val >= min_val && temp_val <= max_val) {
                    value = static_cast<int>(temp_val);
                    return true;
                }
            }
        }
        return false; // Parsing failed or value out of range.
    }

    /**
     * @brief Tokenizes an assembly instruction string.
     * Splits the string by spaces, commas, and parentheses.
     * @param assembly The input assembly instruction string.
     * @return A vector of string tokens.
     */
    static std::vector<std::string> tokenize(const std::string& assembly) {
        std::vector<std::string> tokens;
        std::string current_token;
        for (char c : assembly) {
            // Delimiters are space, comma, and parentheses.
            if (std::isspace(c) || c == ',' || c == '(' || c == ')') {
                if (!current_token.empty()) {
                    tokens.push_back(current_token);
                    current_token.clear();
                }
            }
            else {
                current_token += c; // Append character to current token.
            }
        }
        // Add the last token if any.
        if (!current_token.empty()) {
            tokens.push_back(current_token);
        }
        return tokens;
    }

public:
    /**
     * @brief Decodes a 24-bit machine code instruction (as unsigned int) into its
     *        assembly language string representation.
     * @param instruction The 24-bit machine code.
     * @return The assembly language string.
     */
    static std::string Decode(unsigned int instruction) {
        // Convert the integer machine code to a bit array first.
        std::array<unsigned int, 24> instArr = Convert::ToArray(instruction);
        return Decode(instArr); // Call the array-based Decode.
    }

    /**
     * @brief Decodes a 24-bit machine code instruction (as a bit array) into its
     *        assembly language string representation.
     * Assumes LSB-first bit ordering in the input array.
     * @param instructionArr The 24-bit array representing the machine code.
     * @return The assembly language string (e.g., "add $v0, $v1, $v2").
     */
    static std::string Decode(const std::array<unsigned int, 24>& instructionArr) {
        std::stringstream ss; // Used to build the output string.

        // Extract Opcode (bits 20-23 of the instruction).
        std::array<unsigned int, 4> opcodeArr = Convert::ExtractField<4, 24>(instructionArr, 20);
        unsigned int opcode_val = Convert::ArrayToUInt<4>(opcodeArr);

        // Decode based on Opcode.
        switch (opcode_val) {
        case 0: // R-type instruction
        {
            // Extract R-type fields.
            std::array<unsigned int, 4> rsArr = Convert::ExtractField<4, 24>(instructionArr, 16);
            std::array<unsigned int, 4> rtArr = Convert::ExtractField<4, 24>(instructionArr, 12);
            std::array<unsigned int, 4> rdArr = Convert::ExtractField<4, 24>(instructionArr, 8);
            std::array<unsigned int, 4> shamtArr = Convert::ExtractField<4, 24>(instructionArr, 4);
            std::array<unsigned int, 4> functArr = Convert::ExtractField<4, 24>(instructionArr, 0);

            unsigned int rs_val = Convert::ArrayToUInt<4>(rsArr);
            unsigned int rt_val = Convert::ArrayToUInt<4>(rtArr);
            unsigned int rd_val = Convert::ArrayToUInt<4>(rdArr);
            unsigned int shamt_val = Convert::ArrayToUInt<4>(shamtArr);
            unsigned int funct_val = Convert::ArrayToUInt<4>(functArr);

            std::string rsName = getRegName(rs_val);
            std::string rtName = getRegName(rt_val);
            std::string rdName = getRegName(rd_val);

            // Decode based on Funct field.
            switch (funct_val) {
            case 1: ss << "add " << rdName << ", " << rsName << ", " << rtName; break;
            case 2: ss << "sub " << rdName << ", " << rsName << ", " << rtName; break;
            case 3: ss << "multlow " << rdName << ", " << rsName << ", " << rtName; break;
            case 4: ss << "and " << rdName << ", " << rsName << ", " << rtName; break;
            case 5: ss << "or " << rdName << ", " << rsName << ", " << rtName; break;
            case 6: ss << "xor " << rdName << ", " << rsName << ", " << rtName; break;
            case 7: ss << "slt " << rdName << ", " << rsName << ", " << rtName; break;
            case 8: // sll or nop
                if (rd_val == 0 && rt_val == 0 && shamt_val == 0) { // sll $r0, $r0, 0
                    ss << "nop";
                }
                else {
                    ss << "sll " << rdName << ", " << rtName << ", " << shamt_val;
                }
                break;
            case 9: ss << "srl " << rdName << ", " << rtName << ", " << shamt_val; break;
            default: ss << "Unknown R-type (Funct=" << funct_val << ")"; break;
            }
            break;
        }
        // I-type instructions
        case 1: { // addi rt, rs, immediate
            std::array<unsigned int, 4> rsArr = Convert::ExtractField<4, 24>(instructionArr, 16);
            std::array<unsigned int, 4> rtArr = Convert::ExtractField<4, 24>(instructionArr, 12);
            std::array<unsigned int, 12> immArr = Convert::ExtractField<12, 24>(instructionArr, 0);
            ss << "addi " << getRegName(Convert::ArrayToUInt<4>(rtArr)) << ", "
                << getRegName(Convert::ArrayToUInt<4>(rsArr)) << ", " << formatSignedImm12(immArr);
            break;
        }
        case 2: { // lui rt, immediate
            std::array<unsigned int, 4> rtArr = Convert::ExtractField<4, 24>(instructionArr, 12);
            std::array<unsigned int, 12> immArr = Convert::ExtractField<12, 24>(instructionArr, 0);
            ss << "lui " << getRegName(Convert::ArrayToUInt<4>(rtArr)) << ", " << formatUnsignedImm12Hex(immArr);
            break;
        }
        case 3: { // ori rt, rs, immediate
            std::array<unsigned int, 4> rsArr = Convert::ExtractField<4, 24>(instructionArr, 16);
            std::array<unsigned int, 4> rtArr = Convert::ExtractField<4, 24>(instructionArr, 12);
            std::array<unsigned int, 12> immArr = Convert::ExtractField<12, 24>(instructionArr, 0);
            ss << "ori " << getRegName(Convert::ArrayToUInt<4>(rtArr)) << ", "
                << getRegName(Convert::ArrayToUInt<4>(rsArr)) << ", " << formatUnsignedImm12Hex(immArr);
            break;
        }
        case 4: { // sw rt, offset(rs)
            std::array<unsigned int, 4> rsArr = Convert::ExtractField<4, 24>(instructionArr, 16);
            std::array<unsigned int, 4> rtArr = Convert::ExtractField<4, 24>(instructionArr, 12);
            std::array<unsigned int, 12> immArr = Convert::ExtractField<12, 24>(instructionArr, 0);
            ss << "sw " << getRegName(Convert::ArrayToUInt<4>(rtArr)) << ", "
                << formatSignedImm12(immArr) << "(" << getRegName(Convert::ArrayToUInt<4>(rsArr)) << ")";
            break;
        }
        case 5: { // lw rt, offset(rs)
            std::array<unsigned int, 4> rsArr = Convert::ExtractField<4, 24>(instructionArr, 16);
            std::array<unsigned int, 4> rtArr = Convert::ExtractField<4, 24>(instructionArr, 12);
            std::array<unsigned int, 12> immArr = Convert::ExtractField<12, 24>(instructionArr, 0);
            ss << "lw " << getRegName(Convert::ArrayToUInt<4>(rtArr)) << ", "
                << formatSignedImm12(immArr) << "(" << getRegName(Convert::ArrayToUInt<4>(rsArr)) << ")";
            break;
        }
        case 6: { // bne rs, rt, offset
            std::array<unsigned int, 4> rsArr = Convert::ExtractField<4, 24>(instructionArr, 16);
            std::array<unsigned int, 4> rtArr = Convert::ExtractField<4, 24>(instructionArr, 12);
            std::array<unsigned int, 12> immArr = Convert::ExtractField<12, 24>(instructionArr, 0);
            ss << "bne " << getRegName(Convert::ArrayToUInt<4>(rsArr)) << ", "
                << getRegName(Convert::ArrayToUInt<4>(rtArr)) << ", " << formatSignedImm12(immArr) << " (offset)";
            break;
        }
              // J-type instruction
        case 7: { // j target_address
            std::array<unsigned int, 20> addrArr = Convert::ExtractField<20, 24>(instructionArr, 0);
            ss << "j " << formatUnsignedImm20Hex(addrArr);
            break;
        }
        default: // Unknown opcode
            ss << "Unknown Instruction (Opcode=" << opcode_val << ")";
            break;
        }
        return ss.str();
    }

    /**
     * @brief Encodes an assembly instruction string into its 24-bit machine code representation.
     * @param assembly The assembly instruction string (e.g., "add $v0, $v1, $v2").
     *                 Case-insensitive for mnemonics. Register names are case-sensitive.
     *                 Immediates can be decimal or hex (0x prefix or h suffix).
     * @return The 24-bit machine code as an unsigned integer.
     * @throws std::runtime_error if the assembly string is invalid, uses an unknown
     *         mnemonic, has incorrect operands, or uses invalid register names/immediates.
     */
    static unsigned int Encode(const std::string& assembly) {
        std::vector<std::string> tokens = tokenize(assembly);
        if (tokens.empty()) {
            throw std::runtime_error("Encode Error: Empty assembly string.");
        }

        // Standardize mnemonic to lowercase for map lookup.
        std::string mnemonic = tokens[0];
        std::transform(mnemonic.begin(), mnemonic.end(), mnemonic.begin(),
            [](unsigned char c) { return std::tolower(c); });

        auto it = mnemonicToInfo.find(mnemonic);
        if (it == mnemonicToInfo.end()) {
            throw std::runtime_error("Encode Error: Unknown mnemonic '" + tokens[0] + "'.");
        }

        const InstInfo& info = it->second; // Get encoding info for this mnemonic.
        unsigned int machineCode = 0;
        // Buffers for register numbers and immediate/address values.
        unsigned int rs_val = 0, rt_val = 0, rd_val = 0, shamt_val = 0, imm_val = 0, addr_val = 0;

        // --- Parse Operands and Assemble Machine Code based on Instruction Format ---
        try {
            if (info.format == 'R') { // R-type instructions
                if (mnemonic == "nop") { // Special case: nop is sll $r0, $r0, 0
                    if (tokens.size() - 1 != info.num_operands) throw std::runtime_error("NOP expects 0 operands.");
                    rs_val = 0; rt_val = 0; rd_val = 0; shamt_val = 0;
                    // Funct is already set in info (same as sll).
                }
                else if (mnemonic == "sll" || mnemonic == "srl") { // Format: mnemonic rd, rt, shamt
                    if (tokens.size() - 1 != info.num_operands) throw std::runtime_error(mnemonic + " expects 3 operands (rd, rt, shamt).");
                    if (!getRegNum(tokens[1], rd_val)) throw std::runtime_error("Invalid destination register: " + tokens[1]);
                    if (!getRegNum(tokens[2], rt_val)) throw std::runtime_error("Invalid source register (rt): " + tokens[2]);
                    int temp_shamt;
                    if (!parseImmediate(tokens[3], temp_shamt, 4) || temp_shamt < 0 || temp_shamt > 15) // Shamt is 4 bits (0-15)
                        throw std::runtime_error("Invalid shift amount (0-15): " + tokens[3]);
                    shamt_val = static_cast<unsigned int>(temp_shamt);
                    rs_val = 0; // Rs field is not used for shift amount in MIPS, set to 0.
                }
                else { // Standard R-type format: mnemonic rd, rs, rt
                    if (tokens.size() - 1 != info.num_operands) throw std::runtime_error(mnemonic + " expects 3 operands (rd, rs, rt).");
                    if (!getRegNum(tokens[1], rd_val)) throw std::runtime_error("Invalid destination register: " + tokens[1]);
                    if (!getRegNum(tokens[2], rs_val)) throw std::runtime_error("Invalid source register 1 (rs): " + tokens[2]);
                    if (!getRegNum(tokens[3], rt_val)) throw std::runtime_error("Invalid source register 2 (rt): " + tokens[3]);
                    shamt_val = 0; // Shamt field is 0 for non-shift R-types.
                }
                // Assemble R-type machine code word.
                // Order matches MIPS: Opcode | Rs | Rt | Rd | Shamt | Funct
                machineCode = (info.opcode << 20) | (rs_val << 16) | (rt_val << 12) | (rd_val << 8) | (shamt_val << 4) | info.funct;
            }
            else if (info.format == 'I') { // I-type instructions
                int temp_imm; // Use int for parseImmediate, then cast/mask.
                if (mnemonic == "lui") { // Format: lui rt, immediate
                    if (tokens.size() - 1 != info.num_operands) throw std::runtime_error("lui expects 2 operands (rt, immediate).");
                    if (!getRegNum(tokens[1], rt_val)) throw std::runtime_error("Invalid destination register (rt): " + tokens[1]);
                    if (!parseImmediate(tokens[2], temp_imm, 12)) // Immediate is 12 bits for this custom LUI
                        throw std::runtime_error("Invalid immediate value: " + tokens[2]);
                    imm_val = static_cast<unsigned int>(temp_imm);
                    rs_val = 0; // Rs field is typically 0 for LUI.
                }
                else if (mnemonic == "lw" || mnemonic == "sw") { // Format: mnemonic rt, offset(rs)
                    // Tokenized as: mnemonic, rt, offset, rs
                    if (tokens.size() != 4) throw std::runtime_error(mnemonic + " expects format rt, offset(rs). Actual tokens: " + std::to_string(tokens.size()));
                    if (!getRegNum(tokens[1], rt_val)) throw std::runtime_error("Invalid target/source register (rt): " + tokens[1]);
                    if (!parseImmediate(tokens[2], temp_imm, 12)) // Offset is a 12-bit signed immediate.
                        throw std::runtime_error("Invalid offset value: " + tokens[2]);
                    imm_val = static_cast<unsigned int>(temp_imm);
                    if (!getRegNum(tokens[3], rs_val)) throw std::runtime_error("Invalid base register (rs): " + tokens[3]);
                }
                else if (mnemonic == "bne") { // Format: bne rs, rt, offset
                    if (tokens.size() - 1 != info.num_operands) throw std::runtime_error("bne expects 3 operands (rs, rt, offset).");
                    if (!getRegNum(tokens[1], rs_val)) throw std::runtime_error("Invalid source register 1 (rs): " + tokens[1]);
                    if (!getRegNum(tokens[2], rt_val)) throw std::runtime_error("Invalid source register 2 (rt): " + tokens[2]);
                    if (!parseImmediate(tokens[3], temp_imm, 12)) // Offset is 12-bit signed.
                        throw std::runtime_error("Invalid offset value: " + tokens[3]);
                    imm_val = static_cast<unsigned int>(temp_imm);
                }
                else { // Standard I-type (e.g., addi, ori): mnemonic rt, rs, immediate
                    if (tokens.size() - 1 != info.num_operands) throw std::runtime_error(mnemonic + " expects 3 operands (rt, rs, immediate).");
                    if (!getRegNum(tokens[1], rt_val)) throw std::runtime_error("Invalid destination register (rt): " + tokens[1]);
                    if (!getRegNum(tokens[2], rs_val)) throw std::runtime_error("Invalid source register (rs): " + tokens[2]);
                    if (!parseImmediate(tokens[3], temp_imm, 12)) // Immediate is 12 bits.
                        throw std::runtime_error("Invalid immediate value: " + tokens[3]);
                    imm_val = static_cast<unsigned int>(temp_imm);
                }
                // Assemble I-type machine code word.
                // Order: Opcode | Rs | Rt | Immediate
                machineCode = (info.opcode << 20) | (rs_val << 16) | (rt_val << 12) | (imm_val & 0xFFF); // Mask immediate to 12 bits.
            }
            else if (info.format == 'J') { // J-type instructions: j target_address
                if (tokens.size() - 1 != info.num_operands) throw std::runtime_error("j expects 1 operand (address).");
                int temp_addr;
                if (!parseImmediate(tokens[1], temp_addr, 20)) // Jump address is 20 bits.
                    throw std::runtime_error("Invalid jump target address: " + tokens[1]);
                addr_val = static_cast<unsigned int>(temp_addr);
                // Assemble J-type machine code word.
                // Order: Opcode | Address
                machineCode = (info.opcode << 20) | (addr_val & 0xFFFFF); // Mask address to 20 bits.
            }
        }
        catch (const std::runtime_error& e) {
            // Re-throw parsing/validation errors with more context.
            throw std::runtime_error("Encode Error for '" + assembly + "': " + e.what());
        }
        catch (...) { // Catch any other unexpected errors during parsing.
            throw std::runtime_error("Encode Error for '" + assembly + "': Unknown parsing error.");
        }

        return machineCode;
    }
};





// IF_ID Contents w/R-Inst:     [24-47: PC+3 Address] [20-23: Opcode] [16-19: Rs] [12-15: Rt] [8-11: Rd] [4-7: Shamt] [0-3: Funct]
// ID_EX Contents (92 bits):    [90-91: WB] [88-89: M] [87: RegDst] [85-86: ALUOp] [84: ALUSrc] [60-83: Register 1 Data] [36-59: Register 2 Data] [12-35: Imm Sign Ext] [8-11: Rs] [4-7: Rt] [0-3: Rd]
// EX_MEM Contents (56 bits):   [54-55: WB] [53: MemRead] [52: MemWrite] [28-51: ALU Result/Memory Address] [4-27: Write Data] [0-3: Write Register]
// MEM_WB Contents (54 bits):   [53: RegWrite] [52: MemtoReg] [28-51: Read Data] [4-27: ALU Result] [0-3: Write Register]

// Prints all Pipeline Register states based on LSB=0 indexing.
void PrintPipelineRegisters(PipelineRegisters& PipelineRegistersUnit) {
    // Instruction Format (24 bits):
    // R-type: [20-23: Opcode] [16-19: Rs] [12-15: Rt] [8-11: Rd] [4-7: Shamt] [0-3: Funct]
    // I-type: [20-23: Opcode] [16-19: Rs] [12-15: Rt] [0-11: Constant or Address]
    // J-type: [20-23: Opcode] [0-19: Target Address]

    // --- IF/ID Register ---
    unsigned int opcode = Convert::ArrayToUInt(Convert::ExtractField<4, 48>(PipelineRegistersUnit.IF_ID, 20));
    if (opcode == 0) // R-Type
    {
        std::cout << "IF/ID:    |   PC+3   | Op  Rs  Rt  Rd  Smt Func|" << std::endl;
        // Layout: [0-23: Instruction] [24-47: PC+3 Address]
        std::cout << "  Current | ";
        // PC+3 (Bits 24-47)
        Convert::PrintFieldHex(Convert::ExtractField<24, 48>(PipelineRegistersUnit.IF_ID, 24), 6);
        std::cout << " | ";
        // Instruction Field (Bits 0-23) - Broken down for R-type view
        // Opcode (Bits 20-23 of Instruction -> Indices 20-23 in IF_ID)
        Convert::PrintFieldHex(Convert::ExtractField<4, 48>(PipelineRegistersUnit.IF_ID, 20), 1);
        std::cout << " ";
        // Rs (Bits 16-19 of Instruction -> Indices 16-19 in IF_ID)
        Convert::PrintFieldHex(Convert::ExtractField<4, 48>(PipelineRegistersUnit.IF_ID, 16), 1);
        std::cout << " ";
        // Rt (Bits 12-15 of Instruction -> Indices 12-15 in IF_ID)
        Convert::PrintFieldHex(Convert::ExtractField<4, 48>(PipelineRegistersUnit.IF_ID, 12), 1);
        std::cout << " ";
        // Rd (Bits 8-11 of Instruction -> Indices 8-11 in IF_ID)
        Convert::PrintFieldHex(Convert::ExtractField<4, 48>(PipelineRegistersUnit.IF_ID, 8), 1);
        std::cout << " ";
        // Shamt (Bits 4-7 of Instruction -> Indices 4-7 in IF_ID)
        Convert::PrintFieldHex(Convert::ExtractField<4, 48>(PipelineRegistersUnit.IF_ID, 4), 1);
        std::cout << " ";
        // Funct (Bits 0-3 of Instruction -> Indices 0-3 in IF_ID)
        Convert::PrintFieldHex(Convert::ExtractField<4, 48>(PipelineRegistersUnit.IF_ID, 0), 1);
        std::cout << " |" << std::endl;
    }
    else if (opcode == 0b0111) // J-Type
    {
        std::cout << "IF/ID:    |   PC+3   | Op  |Targ Addr|" << std::endl;
        std::cout << "  Current | ";
        // PC+3 (Bits 24-47)
        Convert::PrintFieldHex(Convert::ExtractField<24, 48>(PipelineRegistersUnit.IF_ID, 24), 6);
        std::cout << " | ";
        // Opcode (Bits 20-23)
        Convert::PrintFieldHex(Convert::ExtractField<4, 48>(PipelineRegistersUnit.IF_ID, 20), 1);
        std::cout << " | ";
        // Target Address (Bits 0-19) - Width 5 hex digits (20 bits / 4)
        Convert::PrintFieldHex(Convert::ExtractField<20, 48>(PipelineRegistersUnit.IF_ID, 0), 5);
        std::cout << " |" << std::endl;
    }
    else // I-Type
    {
        std::cout << "IF/ID:    |   PC+3   | Op  Rs  Rt  |  Imm  |" << std::endl;
        std::cout << "  Current | ";
        // PC+3 (Bits 24-47)
        Convert::PrintFieldHex(Convert::ExtractField<24, 48>(PipelineRegistersUnit.IF_ID, 24), 6);
        std::cout << " | ";
        // Opcode (Bits 20-23)
        Convert::PrintFieldHex(Convert::ExtractField<4, 48>(PipelineRegistersUnit.IF_ID, 20), 1);
        std::cout << " ";
        // Rs (Bits 16-19)
        Convert::PrintFieldHex(Convert::ExtractField<4, 48>(PipelineRegistersUnit.IF_ID, 16), 1);
        std::cout << " ";
        // Rt (Bits 12-15)
        Convert::PrintFieldHex(Convert::ExtractField<4, 48>(PipelineRegistersUnit.IF_ID, 12), 1);
        std::cout << " | ";
        // Immediate (Bits 0-11) - Width 3 hex digits (12 bits / 4)
        Convert::PrintFieldHex(Convert::ExtractField<12, 48>(PipelineRegistersUnit.IF_ID, 0), 3);
        std::cout << " |" << std::endl;
    }


    // --- ID/EX Register ---
    // Layout: [0-3: Rd#] [4-7: Rt#] [8-11: Rs#] [12-35: Imm Sign Ext] [36-59: Reg 2 Data] [60-83: Reg 1 Data]
    //         [84: ALUSrc] [85-86: ALUOp] [87: RegDst] [88-89: M Ctrls] [90-91: WB Ctrls]
    std::cout << "ID/EX:    | WB | M  |    EX     | Reg1 Data| Reg2 Data|ImmSignExt| Rs#| Rt#| Rd#|" << std::endl;
    std::cout << "  Current | ";
    // WB Controls (Bits 90-91)
    Convert::PrintFieldHex(Convert::ExtractField<2, 92>(PipelineRegistersUnit.ID_EX, 90), 1);
    std::cout << "| ";
    // M Controls (Bits 88-89)
    Convert::PrintFieldHex(Convert::ExtractField<2, 92>(PipelineRegistersUnit.ID_EX, 88), 1);
    std::cout << "|";
    // EX Controls: RegDst (Bit 87)
    Convert::PrintFieldHex(Convert::ExtractField<1, 92>(PipelineRegistersUnit.ID_EX, 87), 1);
    std::cout << "|";
    // EX Controls: ALUOp (Bits 85-86)
    Convert::PrintFieldHex(Convert::ExtractField<2, 92>(PipelineRegistersUnit.ID_EX, 85), 1);
    std::cout << "|";
    // EX Controls: ALUSrc (Bit 84)
    Convert::PrintFieldHex(Convert::ExtractField<1, 92>(PipelineRegistersUnit.ID_EX, 84), 1);
    std::cout << "| ";
    // Register 1 Data (Bits 60-83)
    Convert::PrintFieldHex(Convert::ExtractField<24, 92>(PipelineRegistersUnit.ID_EX, 60), 6);
    std::cout << " | ";
    // Register 2 Data (Bits 36-59)
    Convert::PrintFieldHex(Convert::ExtractField<24, 92>(PipelineRegistersUnit.ID_EX, 36), 6);
    std::cout << " | ";
    // Imm Sign Ext (Bits 12-35)
    Convert::PrintFieldHex(Convert::ExtractField<24, 92>(PipelineRegistersUnit.ID_EX, 12), 6);
    std::cout << " | ";
    // Rs# (Bits 8-11)
    Convert::PrintFieldHex(Convert::ExtractField<4, 92>(PipelineRegistersUnit.ID_EX, 8), 1);
    std::cout << "| ";
    // Rt# (Bits 4-7)
    Convert::PrintFieldHex(Convert::ExtractField<4, 92>(PipelineRegistersUnit.ID_EX, 4), 1);
    std::cout << "| ";
    // Rd# (Bits 0-3)
    Convert::PrintFieldHex(Convert::ExtractField<4, 92>(PipelineRegistersUnit.ID_EX, 0), 1);
    std::cout << "|" << std::endl;


    // --- EX/MEM Register ---
    // Layout: [0-3: Write Register#] [4-27: Write Data (for SW)] [28-51: ALU Result/Memory Address]
    //         [52: MemWrite] [53: MemRead] [54-55: WB Ctrls]
    std::cout << "EX/MEM:   | WB |   M   |ALU Result| WriteData|WrReg|" << std::endl;
    std::cout << "  Current | ";
    // WB Controls (Bits 54-55)
    Convert::PrintFieldHex(Convert::ExtractField<2, 56>(PipelineRegistersUnit.EX_MEM, 54), 1);
    std::cout << "|";
    // M Controls: MemRead (Bit 53)
    Convert::PrintFieldHex(Convert::ExtractField<1, 56>(PipelineRegistersUnit.EX_MEM, 53), 1);
    std::cout << "|";
    // M Controls: MemWrite (Bit 52)
    Convert::PrintFieldHex(Convert::ExtractField<1, 56>(PipelineRegistersUnit.EX_MEM, 52), 1);
    std::cout << "| ";
    // ALU Result / Memory Address (Bits 28-51)
    Convert::PrintFieldHex(Convert::ExtractField<24, 56>(PipelineRegistersUnit.EX_MEM, 28), 6);
    std::cout << " | ";
    // Write Data (for SW) (Bits 4-27)
    Convert::PrintFieldHex(Convert::ExtractField<24, 56>(PipelineRegistersUnit.EX_MEM, 4), 6);
    std::cout << " | ";
    // Write Register # (Bits 0-3)
    Convert::PrintFieldHex(Convert::ExtractField<4, 56>(PipelineRegistersUnit.EX_MEM, 0), 1);
    std::cout << " |" << std::endl;

    // --- MEM/WB Register ---
   // Layout: [0-3: Write Register#] [4-27: ALU Result] [28-51: Read Data]
   //         [52: MemtoReg] [53: RegWrite]
    std::cout << "MEM/WB:   | (Wr|MR)| Read Data| ALUResult|WrReg|" << std::endl;
    std::cout << "  Current | ";
    // WB Controls: RegWrite (Bit 53)
    Convert::PrintFieldHex(Convert::ExtractField<1, 54>(PipelineRegistersUnit.MEM_WB, 53), 1);
    std::cout << "|";
    // WB Controls: MemToReg (Bit 52)
    Convert::PrintFieldHex(Convert::ExtractField<1, 54>(PipelineRegistersUnit.MEM_WB, 52), 1);
    std::cout << "| ";
    // Read Data (from Memory) (Bits 28-51)
    Convert::PrintFieldHex(Convert::ExtractField<24, 54>(PipelineRegistersUnit.MEM_WB, 28), 6);
    std::cout << " | ";
    // ALU Result (passed from EX/MEM) (Bits 4-27)
    Convert::PrintFieldHex(Convert::ExtractField<24, 54>(PipelineRegistersUnit.MEM_WB, 4), 6);
    std::cout << " | ";
    // Write Register # (passed from EX/MEM) (Bits 0-3)
    Convert::PrintFieldHex(Convert::ExtractField<4, 54>(PipelineRegistersUnit.MEM_WB, 0), 1);
    std::cout << " |" << std::endl;
}





/**
 * @brief Main simulation driver for a 24-bit MIPS-like pipelined processor.
 *
 * 1. Instantiates all hardware components of the processor.
 * 2. Sets up the interconnections (data paths and control signals) between these components.
 * 3. Initializes the Program Counter (PC), Register File, and Memory with a test program.
 * 4. Runs the simulation for a specified number of clock cycles.
 * 5. Within each cycle, it simulates the combinational logic settling and then the
 *    clock edge update for stateful elements (PC, Pipeline Registers, Register File write).
 * 6. Prints out the state of key components (PC, Pipeline Registers, Register File)
 *    after each clock cycle for debugging and verification.
 *
 * The processor design includes features like a 5-stage pipeline (IF, ID, EX, MEM, WB),
 * forwarding for data hazards, hazard detection for load-use stalls, and branch/jump handling
 * with flushing.
 */
int main() {
    // ========================================================================== //
    // ==========           1. COMPONENT INSTANTIATION             ============== //
    // ========================================================================== //
    // Instantiate all hardware components that make up the processor datapath and control.

    // Control Path Components
    HazardDetector      HazardDetectionUnit("HazardDetectionUnit");
    Controller          ControlUnit("ControlUnit");
    ALUController       ALUControlUnit("ALUControlUnit");
    Forwarder           ForwardingUnit("ForwardingUnit");

    // PC and Fetch Stage Components
    ProgramCounter      PC("PC");
    Memory              MemoryUnit("MemoryUnit"); // Used for both Instruction and Data memory
    Adder               PCIncrementAdder("PCIncrementAdder");

    // Decode Stage Components
    RegisterFile        Registers("Registers");
    SignExtender        SignExtendUnit("SignExtender");
    BranchComparator    BranchCompareUnit("BranchCompareUnit");

    // Execute Stage Components
    ArithmeticLogicUnit ALU("ALU");
    Multiplexor2Way4    WriteRegisterMux("WriteRegisterMux");       // RegDst Mux
    Multiplexor3Way24   ALUForwardAMux("ALUForwardAMux");         // For ALU Input A
    Multiplexor3Way24   ALUForwardBMux("ALUForwardBMux");         // For ALU Input B (before ALUSrc Mux)
    Multiplexor2Way24   ImmediateMux("ImmediateMux");           // ALUSrc Mux (ALU Input B final selection)

    // Memory Stage Components (MemoryUnit is already instantiated)

    // Write-Back Stage Components
    Multiplexor2Way24   WriteDataMux("WriteDataMux");           // MemToReg Mux

    // Branch and Jump Specific Logic
    Multiply3           BranchMulitplyBy3("BranchMulitplyBy3"); // For branch offset scaling
    Adder               BranchAdder("BranchAdder");             // For branch target calculation
    AndGater            BranchAndGate("BranchAndGate");         // For branch taken decision
    OrGater             Flush_IF_ID_OrGate("Flush_IF_ID_OrGate"); // Combines flush signals
    Multiplexor2Way24   BranchAddressMux("BranchAddressMux");   // Selects PC_Next or BranchTarget
    Multiplexor2Way24   JumpMux("JumpMux");                     // Selects BranchMux_Out or JumpTarget
    JumpAddressConcatenator JumpConcat("JumpConcat");           // For J-type address construction
    Multiply3           JumpMultiplyBy3("JumpMultiplyBy3");   // For jump address field scaling

    // ID Stage Forwarding Muxes (for Branch Comparator)
    Multiplexor4Way24   ID_ForwardAMux4Way("ID_ForwardAMux4Way");
    Multiplexor4Way24   ID_ForwardBMux4Way("ID_ForwardBMux4Way");

    // Pipeline Registers and Helper Components
    PipelineRegisters   PipelineRegistersUnit("PipelineRegistersUnit");
    PassThrough         PipelinePassThroughConnections;       // For direct field propagation between stages
    Multiplexor2Way8    ClearControlLinesMultiplexor("ClearControlLinesMultiplexor"); // For NOP insertion

    // ========================================================================== //
    // ==========           2. DATAPATH CONNECTIONS                ============== //
    // ========================================================================== //
    // Define how components are wired together by setting sources for inputs
    // and targets for outputs. Connections are grouped by pipeline stage or function.
    
    // --- Instruction Format and Pipeline Register Layout (Reference) ---
    // These comments are crucial for understanding the bit indices used in connections.
    // Instruction Format (24 bits):
    // R-type: [20-23: Opcode] [16-19: Rs] [12-15: Rt] [8-11: Rd] [4-7: Shamt] [0-3: Funct]
    // I-type: [20-23: Opcode] [16-19: Rs] [12-15: Rt] [0-11: Constant or Address]
    // J-type: [20-23: Opcode] [0-19: Target Address]
    //
    // Pipeline Register Contents (Indices are LSB=0):
    // IF_ID (48 bits):  [0-23: Instruction] [24-47: PC+3 Address]
    // IF_ID Contents w/R-Inst:     [24-47: PC+3 Address] [20-23: Opcode] [16-19: Rs] [12-15: Rt] [8-11: Rd] [4-7: Shamt] [0-3: Funct]
    // ID_EX (92 bits):  [0-3: Rd#] [4-7: Rt#] [8-11: Rs#] [12-35: Imm Sign Ext/Funct/Shamt/Rd]
    //                   [36-59: Register 2 Data] [60-83: Register 1 Data]
    //                   [84: ALUSrc] [85-86: ALUOp] [87: RegDst]
    //                   [88-89: M Ctrls (MemWrite, MemRead)] [90-91: WB Ctrls (MemToReg, RegWrite)]
    // EX_MEM (56 bits): [0-3: Write Register#] [4-27: Write Data (for SW, from Rt)]
    //                   [28-51: ALU Result/Memory Address]
    //                   [52: MemWrite] [53: MemRead]
    //                   [54-55: WB Ctrls (MemToReg, RegWrite)]
    // MEM_WB (54 bits): [0-3: Write Register#] [4-27: ALU Result (from EX/MEM)]
    //                   [28-51: Read Data (from Memory)]
    //                   [52: MemToReg] [53: RegWrite]

    // Instruction Format (24 bits):
    // R-type: [20-23: Opcode] [16-19: Rs] [12-15: Rt] [8-11: Rd] [4-7: Shamt] [0-3: Funct]
    // I-type: [20-23: Opcode] [16-19: Rs] [12-15: Rt] [0-11: Constant or Address]
    // J-type: [20-23: Opcode] [0-19: Target Address]
    // Pipeline Register Contents (Indices are LSB=0):
    // IF_ID Contents w/R-Inst:     [24-47: PC+3 Address] [20-23: Opcode] [16-19: Rs] [12-15: Rt] [8-11: Rd] [4-7: Shamt] [0-3: Funct]
    // ID_EX Contents (92 bits):    [90-91: WB] [88-89: M] [87: RegDst] [85-86: ALUOp] [84: ALUSrc] [60-83: Register 1 Data] [36-59: Register 2 Data] [12-35: Imm Sign Ext] [8-11: Rs] [4-7: Rt] [0-3: Rd]
    // EX_MEM Contents (56 bits):   [54-55: WB] [53: MemRead] [52: MemWrite] [28-51: ALU Result/Memory Address] [4-27: Write Data] [0-3: Write Register]
    // MEM_WB Contents (54 bits):   [53: RegWrite] [52: MemtoReg] [28-51: Read Data] [4-27: ALU Result] [0-3: Write Register]


    ALU.InputShamt_Src.setSource(PipelineRegistersUnit.ID_EX, 16);
    WriteRegisterMux.Selector_Src.setSource(PipelineRegistersUnit.ID_EX, 87, "IDEX_EX[87]->WrRegMux_Sel"); // RegDst
    WriteRegisterMux.InputA_Src.setSource(PipelineRegistersUnit.ID_EX, 4, "IDEX_Rt#->WrRegMux_A");
    WriteRegisterMux.InputB_Src.setSource(PipelineRegistersUnit.ID_EX, 0, "IDEX_Rd#->WrRegMux_B");
    ALUControlUnit.ALUOp_Src.setSource(PipelineRegistersUnit.ID_EX, 85, "IDEX_EX[85-86]->ALUCtrl_ALUOp");
    ALUControlUnit.Funct_Src.setSource(PipelineRegistersUnit.ID_EX, 12, "IDEX_Imm[12-15]->ALUCtrl_Funct"); // Funct is lowest 4 bits of Imm field in ID/EX
    ImmediateMux.Selector_Src.setSource(PipelineRegistersUnit.ID_EX, 84, "IDEX_EX[84]->ImmSrcMux_Sel"); // ALUSrc
    ForwardingUnit.ID_EX_WriteRegister_Src.setSource(WriteRegisterMux.OUTPUT_Internal, "WriteRegMux_Out->FwdUnit_IDEX_WriteReg");
    ForwardingUnit.ID_EX_RegWrite_Src.setSource(PipelineRegistersUnit.ID_EX, 91, "IDEX_WB[1](RegWrite)->FwdUnit_IDEX_RegWrite");
    ForwardingUnit.ID_EX_Rs_Src.setSource(PipelineRegistersUnit.ID_EX, 8, "IDEX_Rs#->Fwd_Rs");
    ForwardingUnit.ID_EX_Rt_Src.setSource(PipelineRegistersUnit.ID_EX, 4, "IDEX_Rt#->Fwd_Rt");
    ALUForwardAMux.InputA_Src.setSource(PipelineRegistersUnit.ID_EX, 60, "IDEX_RD1->ALUFwdAMux_A");
    ALUForwardBMux.InputA_Src.setSource(PipelineRegistersUnit.ID_EX, 36, "IDEX_RD2->ALUFwdBMux_A");
    HazardDetectionUnit.ID_EX_MemRead_Src.setSource(PipelineRegistersUnit.ID_EX, 89, "IDEX_M[1](MemRead)->HDU_IDEX_MemRead");
    HazardDetectionUnit.ID_EX_Rt_Src.setSource(PipelineRegistersUnit.ID_EX, 4, "IDEX_Rt#->HDU_Rt");
    ForwardingUnit.ID_FORWARD_A_MUX_SELECTOR.setTarget(ID_ForwardAMux4Way.Selector_Internal);
    ForwardingUnit.ID_FORWARD_B_MUX_SELECTOR.setTarget(ID_ForwardBMux4Way.Selector_Internal);
    BranchCompareUnit.ReadData1_Src.setSource(ID_ForwardAMux4Way.OUTPUT_Internal);
    BranchCompareUnit.ReadData2_Src.setSource(ID_ForwardBMux4Way.OUTPUT_Internal);
    // ID_Forward A Mux (4-Way) Connections
    ID_ForwardAMux4Way.Selector_Src.setSource(ForwardingUnit.ForwardA_ID_Internal);
    ID_ForwardAMux4Way.InputA_Src.setSource(Registers.READ_DATA_1_OUTPUT_Internal);       // Path 00: RegFile
    ID_ForwardAMux4Way.InputB_Src.setSource(ALU.ALU_RESULT_Internal);                     // Path 01: EX_ALU output
    ID_ForwardAMux4Way.InputC_Src.setSource(PipelineRegistersUnit.EX_MEM, 28);            // Path 10: EX/MEM ALU Result
    ID_ForwardAMux4Way.InputD_Src.setSource(WriteDataMux.OUTPUT_Internal);                // Path 11: MEM/WB Result (via WriteDataMux)
    ID_ForwardAMux4Way.OUTPUT.setTarget(BranchCompareUnit.ReadData1_Internal);
    // ID_Forward B Mux (4-Way) Connections
    ID_ForwardBMux4Way.Selector_Src.setSource(ForwardingUnit.ForwardB_ID_Internal);
    ID_ForwardBMux4Way.InputA_Src.setSource(Registers.READ_DATA_2_OUTPUT_Internal);       // Path 00: RegFile
    ID_ForwardBMux4Way.InputB_Src.setSource(ALU.ALU_RESULT_Internal);                     // Path 01: EX_ALU output
    ID_ForwardBMux4Way.InputC_Src.setSource(PipelineRegistersUnit.EX_MEM, 28);            // Path 10: EX/MEM ALU Result
    ID_ForwardBMux4Way.InputD_Src.setSource(WriteDataMux.OUTPUT_Internal);                // Path 11: MEM/WB Result (via WriteDataMux)
    ID_ForwardBMux4Way.OUTPUT.setTarget(BranchCompareUnit.ReadData2_Internal);
    // Forwarding Unit Connections
    ForwardingUnit.IF_ID_Rs_Src.setSource(PipelineRegistersUnit.IF_ID, 16);
    ForwardingUnit.IF_ID_Rt_Src.setSource(PipelineRegistersUnit.IF_ID, 12);
    ForwardingUnit.EX_MEM_WriteRegister_Src.setSource(PipelineRegistersUnit.EX_MEM, 0, "EXMEM_WrReg#->Fwd_EXMEM_Dst");
    ForwardingUnit.MEM_WB_WriteRegister_Src.setSource(PipelineRegistersUnit.MEM_WB, 0, "MEMWB_WrReg#->Fwd_MEMWB_Dst");
    ForwardingUnit.EX_MEM_RegWrite_Src.setSource(PipelineRegistersUnit.EX_MEM, 55, "EXMEM_WB[1]->Fwd_EXMEM_WrEn");
    ForwardingUnit.MEM_WB_RegWrite_Src.setSource(PipelineRegistersUnit.MEM_WB, 53, "MEMWB_WB[1]->Fwd_MEMWB_WrEn");
    HazardDetectionUnit.IF_ID_Rs_Src.setSource(PipelineRegistersUnit.IF_ID, 16, "IFID_Rs#->HDU_Rs");
    HazardDetectionUnit.IF_ID_Rt_Src.setSource(PipelineRegistersUnit.IF_ID, 12, "IFID_Rt#->HDU_Rt");
    HazardDetectionUnit.PC_WRITE_ENABLE_OUTPUT.setTarget(PC.WriteEnable_Internal, "HDU->PC_WrEn");
    HazardDetectionUnit.IF_ID_WRITE_ENABLE_OUTPUT.setTarget(PipelineRegistersUnit.Write_Enable_IF_ID, "HDU->IFID_WrEn");
    HazardDetectionUnit.CONTROL_MUX_SELECTOR_OUTPUT.setTarget(ClearControlLinesMultiplexor.Selector_Internal, "HDU->CtrlMux_Sel");
    // Control Unit Connections
    ControlUnit.Opcode_Src.setSource(PipelineRegistersUnit.IF_ID, 20, "IFID_Inst[20-23]->Ctrl_Op");
    ControlUnit.PC_SOURCE_OUTPUT.setTarget(BranchAndGate.InputA_Internal, "Ctrl_PCSrc->BranchAND_A");
    ControlUnit.CONTROL_LINES_OUTPUT.setTarget(ClearControlLinesMultiplexor.InputA_Internal, "Ctrl->CtrlMux_A");
    ControlUnit.JUMP_IF_FLUSH_OUTPUT.setTarget(Flush_IF_ID_OrGate.InputA_Internal);
    // Branch AND Gate Connections
    BranchAndGate.InputA_Src.setSource(ControlUnit.PCSource_Internal);
    BranchAndGate.InputB_Src.setSource(BranchCompareUnit.EQUAL_OUTPUT_Internal);
    BranchAndGate.OUTPUT_2.setTarget(Flush_IF_ID_OrGate.InputB_Internal);
    BranchAndGate.OUTPUT.setTarget(BranchAddressMux.Selector_Internal, "BranchAND->PCMux_Sel");
    // Flush IF_ID OR Gate Connections
    Flush_IF_ID_OrGate.InputA_Src.setSource(ControlUnit.Jump_IF_Flush_Internal);
    Flush_IF_ID_OrGate.OUTPUT.setTarget(PipelineRegistersUnit.Flush_IF_ID);
    Flush_IF_ID_OrGate.InputB_Src.setSource(BranchAndGate.OUTPUT_Internal);
    // Branch Adder Connections
    BranchAdder.Augend_Src.setSource(BranchMulitplyBy3.Output24_Internal);
    BranchAdder.Addend_Src.setSource(PipelineRegistersUnit.IF_ID, 24);
    BranchAdder.ADDER_OUTPUT.setTarget(BranchAddressMux.InputB_Internal, "BranchAdd->PCMux_A(BranchTgt)");
    // Branch Address Mux Connections
    BranchAddressMux.Selector_Src.setSource(BranchAndGate.OUTPUT_Internal);
    BranchAddressMux.InputA_Src.setSource(PCIncrementAdder.ADDER_OUTPUT_Internal);
    BranchAddressMux.InputB_Src.setSource(BranchAdder.ADDER_OUTPUT_Internal);
    // Clear Control Lines Multiplexor Connections
    ClearControlLinesMultiplexor.Selector_Src.setSource(HazardDetectionUnit.CONTROL_MUX_SELECT_OUTPUT_Internal);
    ClearControlLinesMultiplexor.InputA_Src.setSource(ControlUnit.Combined_Controls_Output_Internal);
    std::array<unsigned int, 8> HardwireNOP = Convert::ToArray8Bit(0);
    ClearControlLinesMultiplexor.InputB_Src.setSource(HardwireNOP, "Const0->CtrlMux_B");
    ClearControlLinesMultiplexor.OUTPUT.setTarget(PipelineRegistersUnit.ID_EX_Next_Input, 84, "CtrlMux->IDEXNxt_Ctrl");
    // Branch Multiply By 3 Connections
    BranchMulitplyBy3.Input24_Src.setSource(SignExtendUnit.Output_Internal);
    BranchMulitplyBy3.OUTPUT24.setTarget(BranchAdder.Augend_Internal, "LShift2->BranchAdd_A");
    // Other Connections
    PC.WriteEnable_Src.setSource(HazardDetectionUnit.PC_WRITE_OUTPUT_Internal);
    BranchCompareUnit.EQUAL_OUTPUT.setTarget(BranchAndGate.InputB_Internal, "BranchCmp_EQ->BranchAND_B");
    SignExtendUnit.OUTPUT_2.setTarget(BranchMulitplyBy3.Input24_Internal, "SignExt->LShift2_In");
    // Jump Multiplexor Connections
    // Jump Mux Selector
    ControlUnit.JUMP_OUTPUT.setTarget(JumpMux.Selector_Internal, "Target:ControlUnit_Jump->JumpMux_Sel");
    JumpMux.Selector_Src.setSource(ControlUnit.Jump_Internal, "Source:ControlUnit_Jump->JumpMux_Sel");
    // Jump Mux Input A
    BranchAddressMux.OUTPUT.setTarget(JumpMux.InputA_Internal, "Target:BranchMux->JumpMux_A(Seq/Branch)"); // Input 0 = Seq/Branch Target
    JumpMux.InputA_Src.setSource(BranchAddressMux.OUTPUT_Internal, "Source:BranchMux->JumpMux_A(Seq/Branch)");
    // Jump Mux Input B
    JumpConcat.JumpTargetAddress_OUTPUT.setTarget(JumpMux.InputB_Internal, "Target:JumpConcat->JumpMux_B(JumpTgt)"); // Input 1 = Jump Target
    JumpMux.InputB_Src.setSource(JumpConcat.OUTPUT_Internal, "Source:JumpConcat->JumpMux_B(JumpTgt)");
    // Jump Mux Output
    JumpMux.OUTPUT.setTarget(PC.NextValueIn_Internal, "Target:JumpMux_OUT->PC_NextValueIn");
    PC.NextValueIn_Src.setSource(JumpMux.OUTPUT_Internal, "Source:JumpMux_OUT->PC_NextValueIn");
    PCIncrementAdder.MSB2_OUTPUT.setTarget(JumpConcat.PC_MSB2_Internal, "Target:PCAdder_MSB2->JumpConcat_MSB2");
    JumpConcat.PC_MSB2_Src.setSource(PCIncrementAdder.MSB2_OUTPUT_Internal, "Source:PCAdder_MSB2->JumpConcat_MSB2");
    // Jump Concat 22 Bit LSB Input
    JumpMultiplyBy3.OUTPUT22.setTarget(JumpConcat.ShiftedAddress_Internal, "Target:JumpLeftShift_Out22->JumpConcat_In22");
    JumpConcat.ShiftedAddress_Src.setSource(JumpMultiplyBy3.Output22_Internal, "Source:JumpLeftShift_Out22->JumpConcat_In22");
    JumpMultiplyBy3.Input20_Src.setSource(PipelineRegistersUnit.IF_ID, 0, "Source:IF_ID[0-11]->JumpLeftShift_In20");
    // Direct Pipeline Interconnections
    PipelinePassThroughConnections.IFID_TO_IDEX_RS_Src.setSource(PipelineRegistersUnit.IF_ID, 16);
    PipelinePassThroughConnections.IFID_TO_IDEX_RT_Src.setSource(PipelineRegistersUnit.IF_ID, 12);
    PipelinePassThroughConnections.IFID_TO_IDEX_RD_Src.setSource(PipelineRegistersUnit.IF_ID, 8);
    PipelinePassThroughConnections.IDEX_TO_EXMEM_WB_Src.setSource(PipelineRegistersUnit.ID_EX, 90);
    PipelinePassThroughConnections.IDEX_TO_EXMEM_M_Src.setSource(PipelineRegistersUnit.ID_EX, 88);
    PipelinePassThroughConnections.EXMEM_TO_MEMWB_WB_Src.setSource(PipelineRegistersUnit.EX_MEM, 54);
    PipelinePassThroughConnections.EXMEM_TO_MEMWB_WRITE_REGISTER_Src.setSource(PipelineRegistersUnit.EX_MEM, 0);
    PipelinePassThroughConnections.EXMEM_TO_MEMWB_ALU_RESULT_Src.setSource(PipelineRegistersUnit.EX_MEM, 28);
    PipelinePassThroughConnections.IFID_TO_IDEX_RS_OUTPUT.setTarget(PipelineRegistersUnit.ID_EX_Next_Input, 8);
    PipelinePassThroughConnections.IFID_TO_IDEX_RT_OUTPUT.setTarget(PipelineRegistersUnit.ID_EX_Next_Input, 4);
    PipelinePassThroughConnections.IFID_TO_IDEX_RD_OUTPUT.setTarget(PipelineRegistersUnit.ID_EX_Next_Input, 0);
    PipelinePassThroughConnections.IDEX_TO_EXMEM_WB_OUTPUT.setTarget(PipelineRegistersUnit.EX_MEM_Next_Input, 54);
    PipelinePassThroughConnections.IDEX_TO_EXMEM_M_OUTPUT.setTarget(PipelineRegistersUnit.EX_MEM_Next_Input, 52);
    PipelinePassThroughConnections.EXMEM_TO_MEMWB_WB_OUTPUT.setTarget(PipelineRegistersUnit.MEM_WB_Next_Input, 52);
    PipelinePassThroughConnections.EXMEM_TO_MEMWB_WRITE_REGISTER_OUTPUT.setTarget(PipelineRegistersUnit.MEM_WB_Next_Input, 0);
    PipelinePassThroughConnections.EXMEM_TO_MEMWB_ALU_RESULT_OUTPUT.setTarget(PipelineRegistersUnit.MEM_WB_Next_Input, 4);
    MemoryUnit.InstAddress_Src.setSource(PC.Value);
    PCIncrementAdder.Augend_Src.setSource(PC.Value);
    ImmediateMux.InputA_Src.setSource(ALUForwardBMux.OUTPUT_Internal);
    ALU.ALUControlLines_Src.setSource(ALUControlUnit.ALU_CONTROL_Internal);
    ALU.InputA_Src.setSource(ALUForwardAMux.OUTPUT_Internal);
    ALU.InputB_Src.setSource(ImmediateMux.OUTPUT_Internal);
    Registers.WriteData_Src.setSource(WriteDataMux.OUTPUT_Internal);
    // --- IF Stage Connections ---
    PC.OUT1.setTarget(MemoryUnit.InstAddress_Internal, "PC->MemAddr");
    PC.OUT2.setTarget(PCIncrementAdder.Augend_Internal, "PC->PCIncAdd_A");
    std::array<unsigned int, 24> HardwireValue3 = Convert::ToArray(3);
    PCIncrementAdder.Addend_Src.setSource(HardwireValue3, "Const3->PCIncAdd_B");
    PCIncrementAdder.ADDER_OUTPUT.setTarget(PipelineRegistersUnit.IF_ID_Next_Input, 24, "PCIncAdd->IFIDNxt_PC+3");
    PCIncrementAdder.ADDER_OUTPUT_2.setTarget(BranchAddressMux.InputA_Internal); // <------------ branch target
    MemoryUnit.INSTRUCTION_OUTPUT.setTarget(PipelineRegistersUnit.IF_ID_Next_Input, 0, "Mem->IFIDNxt_Inst");
    // --- ID Stage Connections ---
    Registers.ReadRegister1_Src.setSource(PipelineRegistersUnit.IF_ID, 16, "IFID_Inst[16-19]->Reg_Read1#");
    Registers.ReadRegister2_Src.setSource(PipelineRegistersUnit.IF_ID, 12, "IFID_Inst[12-15]->Reg_Read2#");
    SignExtendUnit.Input_Src.setSource(PipelineRegistersUnit.IF_ID, 0, "IFID_Inst[0-11]->SignExt_In");
    Registers.READ_DATA_1_OUTPUT.setTarget(PipelineRegistersUnit.ID_EX_Next_Input, 60, "Reg_Read1Data->IDEXNxt_RD1");
    Registers.READ_DATA_2_OUTPUT.setTarget(PipelineRegistersUnit.ID_EX_Next_Input, 36, "Reg_Read2Data->IDEXNxt_RD2");
    SignExtendUnit.OUTPUT_1.setTarget(PipelineRegistersUnit.ID_EX_Next_Input, 12, "SignExt->IDEXNxt_Imm");
    // --- EX Stage Connections ---
    // Forwarding Mux Connections
    ForwardingUnit.ALU_FORWARD_A_MUX_SELECTOR.setTarget(ALUForwardAMux.Selector3Way_Internal, "Fwd_A->ALUFwdAMux_Sel");
    ForwardingUnit.ALU_FORWARD_B_MUX_SELECTOR.setTarget(ALUForwardBMux.Selector3Way_Internal, "Fwd_B->ALUFwdBMux_Sel");
    ALUForwardAMux.Selector3Way_Src.setSource(ForwardingUnit.ForwardA_EX_Internal);
    ALUForwardAMux.InputB_Src.setSource(WriteDataMux.OUTPUT_Internal, "WB_WrData->ALUFwdAMux_B"); // WB Data
    ALUForwardAMux.InputC_Src.setSource(PipelineRegistersUnit.EX_MEM, 28, "EXMEM_ALURes->ALUFwdAMux_C"); // MEM Data (ALU Res)
    ALUForwardBMux.Selector3Way_Src.setSource(ForwardingUnit.ForwardB_EX_Internal);
    ALUForwardBMux.InputB_Src.setSource(WriteDataMux.OUTPUT_Internal, "WB_WrData->ALUFwdBMux_B"); // WB Data
    ALUForwardBMux.InputC_Src.setSource(PipelineRegistersUnit.EX_MEM, 28, "EXMEM_ALURes->ALUFwdBMux_C"); // MEM Data (ALU Res)
    ALUForwardBMux.OUTPUT_2.setTarget(PipelineRegistersUnit.EX_MEM_Next_Input, 4, "ALUFwdBMux->EXMEM_Write_Data");
    // ALU Input Connections
    ALUForwardAMux.OUTPUT.setTarget(ALU.InputA_Internal, "ALUFwdAMux->ALU_A");
    ALUForwardBMux.OUTPUT.setTarget(ImmediateMux.InputA_Internal, "ALUFwdBMux->ImmSrcMux_A"); // Reg Data 2 (Fwd)
    ImmediateMux.InputB_Src.setSource(PipelineRegistersUnit.ID_EX, 12, "IDEX_Imm->ImmSrcMux_B"); // Immediate
    ImmediateMux.OUTPUT.setTarget(ALU.InputB_Internal, "ImmSrcMux->ALU_B");
    ALUControlUnit.ALU_CONTROL_OUTPUT.setTarget(ALU.ALUControlLines_Internal, "ALUCtrl->ALU_Ctrl");
    // Outputs to EX/MEM Next Buffer
    ALU.ALU_RESULT.setTarget(PipelineRegistersUnit.EX_MEM_Next_Input, 28, "ALU->EXMEMNxt_ALURes");
    WriteRegisterMux.OUTPUT.setTarget(PipelineRegistersUnit.EX_MEM_Next_Input, 0, "WrRegMux->EXMEMNxt_WrReg#");
    // --- MEM Stage Connections ---
    MemoryUnit.MemRead_Src.setSource(PipelineRegistersUnit.EX_MEM, 53, "EXMEM_M[1]->Mem_ReadEn");
    MemoryUnit.MemWrite_Src.setSource(PipelineRegistersUnit.EX_MEM, 52, "EXMEM_M[0]->Mem_WriteEn");
    MemoryUnit.MemAddress_Src.setSource(PipelineRegistersUnit.EX_MEM, 28, "EXMEM_ALURes->Mem_Addr");
    MemoryUnit.WriteData_Src.setSource(PipelineRegistersUnit.EX_MEM, 4, "EXMEM_WrData->Mem_WrData");
    MemoryUnit.READ_DATA_OUTPUT.setTarget(PipelineRegistersUnit.MEM_WB_Next_Input, 28, "Mem_ReadData->MEMWBNxt_RD");
    // --- WB Stage Connections ---
    WriteDataMux.Selector_Src.setSource(PipelineRegistersUnit.MEM_WB, 52, "MEMWB_WB[1]->WrDataMux_Sel"); // MemToReg
    WriteDataMux.InputA_Src.setSource(PipelineRegistersUnit.MEM_WB, 4, "MEMWB_ALURes->WrDataMux_A"); // ALU Result
    WriteDataMux.InputB_Src.setSource(PipelineRegistersUnit.MEM_WB, 28, "MEMWB_ReadData->WrDataMux_B"); // Memory Data
    Registers.RegWrite_Src.setSource(PipelineRegistersUnit.MEM_WB, 53, "MEMWB_WB[0]->Reg_WrEn");
    Registers.WriteRegister_Src.setSource(PipelineRegistersUnit.MEM_WB, 0, "MEMWB_WrReg#->Reg_Wr#");
    WriteDataMux.OUTPUT.setTarget(Registers.WriteData_Next_Input, "WrDataMux->Reg_WrData");
    WriteDataMux.OUTPUT_2.setTarget(ALUForwardAMux.InputB_Internal, "WrDataMux->ALUFwdAMux_B(WB) Optional"); // WB->EX Forwarding Path A
    WriteDataMux.OUTPUT_3.setTarget(ALUForwardBMux.InputB_Internal, "WrDataMux->ALUFwdBMux_B(WB) Optional"); // WB->EX Forwarding Path B

    // ========================================================================== //
    // ==========           3. SIMULATION SETUP & EXECUTION        ============== //
    // ========================================================================== //
    
    // --- Simulation Output Control Variables ---
    bool showPipelineRegisterValues = true;
    bool showRegisterFileValues = true;
    bool showDataMemoryValues = true;
    int startCycle = 0;
    int endCycle = 77; // Default total simulation cycles

    // --- User Interaction for Simulation Control ---
    char choice;
    std::cout << "--- Simulation Configuration ---" << std::endl;

    std::cout << "Show Pipeline Register values? (y/n, default y): ";
    std::cin >> choice;
    if (choice == 'n' || choice == 'N') showPipelineRegisterValues = false;
    else if (choice != 'y' && choice != 'Y') { /* Keep default or handle invalid */ }
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Consume newline

    std::cout << "Show Register File values? (y/n, default y): ";
    std::cin >> choice;
    if (choice == 'n' || choice == 'N') showRegisterFileValues = false;
    else if (choice != 'y' && choice != 'Y') { /* Keep default or handle invalid */ }
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    std::cout << "Show Data Memory values? (y/n, default y): ";
    std::cin >> choice;
    if (choice == 'n' || choice == 'N') showDataMemoryValues = false;
    else if (choice != 'y' && choice != 'Y') { /* Keep default or handle invalid */ }
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    int inputStartCycle, inputEndCycle;
    std::cout << "Enter start cycle for detailed output (default 0): ";
    std::string line;
    std::getline(std::cin, line);
    if (!line.empty()) {
        try {
            inputStartCycle = std::stoi(line);
            if (inputStartCycle >= 0) startCycle = inputStartCycle;
            else std::cout << "Invalid start cycle, using default " << startCycle << std::endl;
        }
        catch (const std::exception& e) {
            std::cout << "Invalid input for start cycle, using default " << startCycle << std::endl;
        }
    }

    std::cout << "Enter end cycle for detailed output (test program ends at 77): ";
    std::getline(std::cin, line);
    if (!line.empty()) {
        try {
            inputEndCycle = std::stoi(line);
            if (inputEndCycle >= startCycle && inputEndCycle <= endCycle) { // Ensure end cycle is valid
                // endCycle remains the total simulation cycles
                // We'll use inputEndCycle for the print range
            }
            else if (inputEndCycle < startCycle) {
                std::cout << "End cycle cannot be less than start cycle. Using default range." << std::endl;
                // Keep default startCycle and default endCycle for printing by not changing them
                // Or, set inputEndCycle = endCycle; to print up to max.
                inputEndCycle = endCycle; // For clarity in the print condition later
            }
            else if (inputEndCycle > endCycle) {
                std::cout << "End cycle cannot exceed total simulation cycles (" << endCycle << "). Clamping to max." << std::endl;
                inputEndCycle = endCycle;
            }
        }
        catch (const std::exception& e) {
            std::cout << "Invalid input for end cycle, using default range up to " << endCycle << std::endl;
            inputEndCycle = endCycle; // Set for print condition
        }
    }
    else {
        inputEndCycle = endCycle; // If no input, use default total simulation cycles for printing range
    }
    // `endCycle` still controls the total simulation length
    // `inputEndCycle` will control the print range alongside `startCycle`.



    // --- List of All Driveable Units for the Simulation Loop ---
    // The order in this list can be important if combinational_iterations = 1.
    // For multiple iterations, it mainly affects how quickly the system settles.
    // A generally good order:
    // 1. Units that primarily read from stable state (PC, Pipeline Registers, RegFile read) and generate early signals.
    // 2. Units that depend on outputs of (1).
    // 3. Complex units like Forwarder and HazardDetector that need many inputs to be stable.
    // 4. Muxes that take selectors from (3).
    // 5. Units that take outputs from those muxes.
    // PipelinePassThroughConnections can run early to propagate fields.
    // PipelineRegistersUnit is special and handled at the "clock edge".
    std::vector<DriveableUnit*> simulation_units = {
        // Units typically driven early in the combinational phase
        &PC,                            // To make PC.Value available (WriteOutput)
        &MemoryUnit,                    // Instruction Fetch (uses PC.Value), Data Access (uses EX/MEM)
        &PCIncrementAdder,              // Uses PC.Value
        &Registers,                     // Read ports (uses IF/ID)
        &ControlUnit,                   // Uses IF/ID
        &SignExtendUnit,                // Uses IF/ID
        &JumpMultiplyBy3,               // Uses IF/ID
        &BranchMulitplyBy3,             // Uses SignExtendUnit output
        &BranchAdder,                   // Uses BranchMultiplyBy3, IF/ID
        &JumpConcat,                    // Uses JumpMultiplyBy3, PCIncrementAdder
        &PipelinePassThroughConnections,// Propagates fields between stages

        // EX Stage logic (relies on stable ID/EX)
        &WriteRegisterMux,
        &ALUControlUnit,
        // ALU and its input muxes - Forwarder selectors are needed first
        // Forwarder depends on many stable inputs, including WriteRegisterMux output

        // ID Stage branch logic (relies on RegFile, Forwarder, ALU output for EX->ID)
        // The order of ALU, Forwarder, ID_ForwardMuxes, BranchCompareUnit is important
        // ALU must run to provide current result.
        // Forwarder must run to select if current ALU result is needed.
        // ID_ForwardMuxes use Forwarder output and ALU result.

        // This order attempts to resolve dependencies:
        &ALUForwardAMux,                // Needs Forwarder output for selector
        &ALUForwardBMux,                // Needs Forwarder output for selector
        &ImmediateMux,                  // Needs ALUForwardBMux output
        &ALU,                           // Needs ALUForwardAMux, ImmediateMux outputs

        &ID_ForwardAMux4Way,            // Needs Forwarder output for selector & ALU output
        &ID_ForwardBMux4Way,            // Needs Forwarder output for selector & ALU output
        &BranchCompareUnit,             // Needs ID_ForwardMux outputs

        // Logic using branch condition
        &BranchAndGate,
        &Flush_IF_ID_OrGate,
        &BranchAddressMux,
        &JumpMux,

        // Units that determine stalls/control overrides (need widespread state)
        &HazardDetectionUnit,
        &ClearControlLinesMultiplexor,  // Uses HazardDetectionUnit output

        // ForwardingUnit should run after its sources are stable, including WriteRegisterMux.OUTPUT and ALU.ALU_RESULT
        &ForwardingUnit,

        // WB stage mux
        &WriteDataMux,

        // PipelineRegisters is stateful, handled separately at clock edge.
        // &PipelineRegistersUnit (not in this list for DriveUnit loop, handled at clock edge)
    };

    // --- Test Program Initialization ---
    //std::cout << "--- Initializing Test ---" << std::endl;
    PC.Value = Convert::ToArray(0); // Set initial PC
    //std::cout << "Initial PC Value: 0x" << std::hex << Convert::ArrayToUInt<24>(PC.Value) << std::dec << std::endl;

    // Initialize Register File
    Registers.Registers[0] = Convert::ToArray(0x000000); // $r0
    Registers.Registers[1] = Convert::ToArray(0x000001); // $v0
    Registers.Registers[2] = Convert::ToArray(0x000001); // $v1
    Registers.Registers[3] = Convert::ToArray(0x0000FF); // $v2
    Registers.Registers[4] = Convert::ToArray(0x00000F); // $v3
    Registers.Registers[5] = Convert::ToArray(0x000000); // $t0
    Registers.Registers[6] = Convert::ToArray(0x000051); // $a0
    Registers.Registers[7] = Convert::ToArray(0x000005); // $a1
    Registers.Registers[8] = Convert::ToArray(0x000000); // $at

    // Define and Load Test Program into Memory
    std::vector<std::pair<unsigned int, std::string>> program = {



        {0x00, "addi $a1, $a1, -1"},        // cycle 5: a1 = 4          cycle 19: a1 = 3        cycle 33: a1 = 2        cycle 47: a1 = 1            cycle 62: a1 = 0
        {0x03, "lw $t0, 0($a0)"},           // cycle 6: t0 = 1          cycle 20: t0 = 3        cycle 34: t0 = 5        cycle 48: t0 = 0xFFFFFF     cycle 63: t0 = 0xFFFFFF
        {0x06, "addi $a0, $a0, 3"},         // cycle 7: a0 = 0x54       cycle 21: a0 = 0x57     cycle 35: a0 = 0x5A     cycle 49: a0 = 0x5D         cycle 64: a0 = 0x60
        {0x09, "slt $at, $r0, $t0"},        // cycle 8: at = 1          cycle 22: at = 1        cycle 36: at = 1        cycle 50: at = 0            cycle 65: at = 0
        {0x0C, "bne $at, $r0, 0x6"},        // cycle 9: jump to 0x21    cycle 23: jump to 0x21  cycle 37: jump to 0x21  cycle 51: don't branch      cycle XX: don't branch
        {0x0F, "srl $v2, $v2, 1"},          //                                                                          cycle 52: v2 = 0x7F         cycle XX: v2 = 0x3F
        {0x12, "sub $v3, $v3, $t0"},        //                                                                          cycle 53: v3 = 0x10         cycle XX: v3 = 0x11
        {0x15, "lui $at, 0xFFF"},           //                                                                          cycle 54: at = 0xFFF000     cycle XX: at = 0xFFF000
        {0x18, "ori $at, $at, 0xF00"},      //                                                                          cycle 55: at = 0xFFFF00     cycle XX: at = 0xFFFF00
        {0x1B, "sw $at, -3($a0)"},          //                                                                          cycle 56: SW inst.          cycle XX: SW inst.
        {0x1E, "j 0x10"},                   //                                                                          cycle 57: jump to 0x30      cycle XX: jump to 0x30
        {0x21, "addi $at, $r0, 3"},         // cycle 11: at = 3         cycle 25: at = 3        cycle 39: at = 3
        {0x24, "multlow $v0, $v0, $at"},    // cycle 12: v0 = 3         cycle 26: v0 = 9        cycle 40: v0 = 0x1B
        {0x27, "xor $v1, $v1, $t0"},        // cycle 13: v1 = 0         cycle 27: v1 = 3        cycle 41: v1 = 6
        {0x2A, "addi $at, $r0, 0x0FF"},     // cycle 14: at = 0x0FF     cycle 28: at = 0x0FF    cycle 42: at = 0x0FF
        {0x2D, "sw $at, -3($a0)"},          // cycle 15: SW inst.       cycle 29: SW inst.      cycle 43: SW inst.
        {0x30, "slt $at, $r0, $a1"},        // cycle 16: at = 1         cycle 30: at = 1        cycle 44: at = 1        cycle XX: at = 1            cycle XX: at = 0
        {0x33, "bne $at, $r0, 0xFEE"}       // cycle XX: jump to 0x00   cycle XX: jump to 0x00  cycle XX: jump to 0x00  cycle XX: jump to 0x00      cycle XX: don't branch
    
    
    
    };
    // Final Register Values:
    // $r0 = 0x0
    // $v0 = 0x1B
    // $v1 = 0x6
    // $v2 = 0x3F
    // $v3 = 0x11
    // $t0 = 0xFFFFFF
    // $a0 = 0x60
    // $a1 = 0x0
    // $at = 0x0


    // Initialize some data memory locations for lw/sw
    MemoryUnit.MemoryFile[0x51] = Convert::ToArray(0x000001); // Data for lw $t0, 0($a0)
    MemoryUnit.MemoryFile[0x54] = Convert::ToArray(0x000003); // Data for lw if $a0 becomes 0x53
    MemoryUnit.MemoryFile[0x57] = Convert::ToArray(0x000005); // Data for lw if $a0 becomes 0x56
    MemoryUnit.MemoryFile[0x5A] = Convert::ToArray(0xFFFFFF); // Data for lw if $a0 becomes 0x59
    MemoryUnit.MemoryFile[0x5D] = Convert::ToArray(0xFFFFFF); // Data for lw if $a0 becomes 0x5C

    for (const auto& instruction_pair : program) {
        unsigned int address = instruction_pair.first;
        const std::string& test_asm = instruction_pair.second;
        try {
            unsigned int mc = InstructionDecoder::Encode(test_asm);
            std::string decoded_asm = InstructionDecoder::Decode(mc);
            //std::cout << "  Addr 0x" << std::hex << std::setw(2) << std::setfill('0') << address << ": "
            //    << std::setw(25) << std::left << std::setfill(' ') << test_asm << " -> 0x"
            //    << std::hex << std::right << std::setw(6) << std::setfill('0') << mc
            //    << " -> " << decoded_asm << std::dec << std::setfill(' ') << std::endl;
            if (address < MemoryUnit.MemoryFile.size()) {
                MemoryUnit.MemoryFile[address] = Convert::ToArray(mc);
            }
            else {
                std::cerr << "Error: Address 0x" << std::hex << address << std::dec
                    << " is out of memory bounds for instruction '" << test_asm << "'." << std::endl;
            }
        }
        catch (const std::runtime_error& e) {
            std::cerr << "Error encoding '" << test_asm << "': " << e.what() << std::endl;
        }
    }

    // --- Simulation Loop ---
    int sim_cycles = endCycle; // Number of clock cycles to simulate
    for (int cycle = 1; cycle <= sim_cycles; ++cycle) {
        if (cycle >= startCycle && cycle <= inputEndCycle) { // Check against user-defined print range
            std::cout << "\n--- Simulating Cycle " << cycle << " ---" << std::endl;
        }

        // --- Combinational Logic Phase ---
        // Simulate signal propagation and settling.
        // Multiple iterations allow signals to propagate through chains of combinational logic.
        const int combinational_iterations = 8;
        for (int i = 0; i < combinational_iterations; ++i) {
            // Drive all combinational units. The order in `simulation_units` helps guide
            // the flow of data if iterations were 1, but with multiple iterations,
            // the system should settle regardless of precise order within the list.
            // Stateful elements (PC, PipelineRegisters) are not driven for their latching action here.
            // RegisterFile.DriveUnit handles both read (combinational) and prepares for write (clocked).
            // MemoryUnit.DriveUnit handles IF (combinational on PC) and MEM access (combinational on EX/MEM inputs).

            // Explicitly drive Register File reads and Memory IF/MEM operations once per combinational phase.
            if (i == 0) {
                PC.WriteOutput(); // Make current PC value available on its output ports.
                Registers.DriveUnit(i); // Perform register reads based on IF/ID.
                Registers.WriteOutput();    // Propagate read data.
                MemoryUnit.DriveUnit(i); // Perform IF and Memory access logic based on current inputs.
                MemoryUnit.WriteOutput();   // Propagate IF/MEM results.
            }

            // Drive other combinational units.
            for (auto unit_ptr : simulation_units) {
                // Skip state elements and those driven explicitly above in the first iteration.
                if (unit_ptr == &PC || unit_ptr == &PipelineRegistersUnit ||
                    (i == 0 && (unit_ptr == &Registers || unit_ptr == &MemoryUnit))) {
                    continue;
                }
                unit_ptr->DriveUnit(i);   // Simulate combinational logic.
                unit_ptr->WriteOutput();  // Propagate outputs to _Internal buffers of next components or _Next_Input of registers.
            }
        } // End combinational_iterations loop

        // --- Clock Edge Phase ---
        // Update stateful elements: PC, Pipeline Registers, Register File (write part).
        PC.DriveUnit(); // Latches PC.NextValueIn_Internal into PC.Value if enabled.
        PipelineRegistersUnit.DriveUnit(); // Latches all _Next_Input buffers into IF/ID, ID/EX, etc.
        // Also resets IF/ID stall/flush controls for the next cycle.
        Registers.DriveUnit(); // This call primarily latches the write-side inputs
        // (WriteReg#, WriteData, RegWrite from _Src into _Next_Input,
        // then _Next_Input into _Internal for PerformWrite).

        // --- Perform Actual State Writes (Post-Clock Edge Behavior) ---
        Registers.PerformWrite(); // Performs the write to the register array if RegWrite is enabled.

        // --- Print Post-Clock Edge State based on user preferences and cycle range ---
        if (cycle >= startCycle && cycle <= inputEndCycle) {
            std::cout << "  Current PC: "; Convert::PrintArray(PC.Value);

            if (showPipelineRegisterValues) {
                std::cout << "  Pipeline Registers State:" << std::endl;
                PrintPipelineRegisters(PipelineRegistersUnit);
            }
            if (showRegisterFileValues) {
                std::cout << "  Register File State:" << std::endl;
                Registers.PrintRegisterFileState("    ");
            }
            if (showDataMemoryValues) {
                // Optionally, you might want to limit the range of memory printed
                // or only print if changes occurred, but for now, this matches the original.
                std::cout << "  Data Memory (relevant range):" << std::endl;
                MemoryUnit.PrintMemoryState(false, 0x51, 0x5d, "    ");
            }
            std::cout << std::endl; // Add a blank line after each detailed cycle print
        }
        // Special print for the very last cycle if it's outside the user's detailed range
        // but they still want to see the final state (matches original behavior slightly modified)
        else if (cycle == sim_cycles && (cycle < startCycle || cycle > inputEndCycle)) {
            std::cout << "\n--- Final State (Cycle " << cycle << ") ---" << std::endl;
            std::cout << "  Current PC: "; Convert::PrintArray(PC.Value);
            if (showPipelineRegisterValues) {
                std::cout << "  Pipeline Registers State:" << std::endl;
                PrintPipelineRegisters(PipelineRegistersUnit);
            }
            if (showRegisterFileValues) {
                std::cout << "  Register File State:" << std::endl;
                Registers.PrintRegisterFileState("    ");
            }
            if (showDataMemoryValues) {
                std::cout << "  Data Memory (relevant range):" << std::endl;
                MemoryUnit.PrintMemoryState(false, 0x51, 0x5d, "    ");
            }
            std::cout << std::endl;
        }




    } // End simulation_cycles loop


    std::cout << "\n--- Simulation Complete ---" << std::endl;
    // Final state if detailed view didn't cover the last cycle.
    // This ensures the very last state is shown if the loop range for detailed printing was shorter.
    if (!(sim_cycles >= startCycle && sim_cycles <= inputEndCycle)) {
        std::cout << "\n--- Final State (Cycle " << sim_cycles << ") Summary ---" << std::endl;
        if (showRegisterFileValues) Registers.PrintRegisterFileState("    ");
        if (showDataMemoryValues) MemoryUnit.PrintMemoryState(false, 0x51, 0x5d, "    ");
        std::cout << std::endl;
    }

    return 0;


};


