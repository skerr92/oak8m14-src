
// Source: https://stackoverflow.com/a/31302223/83062366
`define assert(signal, value) \
        if (signal !== value) begin \
            $error("ASSERTION FAILED in %m: signal != value"); \
            $fatal(); \
        end