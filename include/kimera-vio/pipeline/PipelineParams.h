#pragma once

#include <string>

namespace VIO{


class PipelineParams{
public:
    explicit PipelineParams(const std::string& name);
    virtual ~PipelineParams() = default;
public:
    virtual bool parseYAML(const std::string& filepath) = 0;

    virtual void print() const = 0;

    friend bool operator==(const PipelineParams& lhs, const PipelineParams& rhs);
    friend bool operator!=(const PipelineParams& lhs, const PipelineParams& rhs);

protected:
    virtual bool equals(const PipelineParams& obj) const = 0;

    template < typename... args>
    void print(std::stringstream& out, args... to_print) const {};

public:
    std::string name_;
    static constexpr size_t kTotalWidth = 60;
    static constexpr size_t kNameWidth = 40;
    static constexpr size_t kValueWidth = 20;

private:
    template <typename TName, typename TValue>
    void printImpl(std::stringstream& out, TName name, TValue value) const {}

    template <typename TName, typename TValue, typename... Args>
    void printImpl(std::stringstream& out, TName name, TValue value) const {}


};

inline bool operator==(const PipelineParams& lhs, const PipelineParams& rhs) {}
inline bool operator!=(const PipelineParams& lhs, const PipelineParams& rhs) {}


} // namespace VIO