require 'pry'
require 'active_support/all'

CTYPES_MAP = {
  'int' => 'c_int',
  # 'mjContact' => 'c_void_p',
  'double' => 'c_double',
  'float' => 'c_float',
  'char' => 'c_char',
  'unsigned char' => 'c_ubyte',
  'unsigned int' => 'c_uint',
}

CTYPES_PTR_MAP = {
  'void' => 'c_void_p',
  # 'char' => 'c_char_p',
}

DEDEF_MAP = {
  'mjtNum' => 'double',
  'mjtByte' => 'unsigned char',
  # 'mjNREF' => '2',
  # 'mjNDYN' => '3',
  # 'mjNGAIN' => '3',
  # 'mjNBIAS' => '3',
  # 'mjNIMP' => '3',
  # 'mjNEQDATA' => '7',
  # 'mjNTRN' => '1',
}

NP_DTYPE_MAP = {
  'double' => 'double',
  'float' => 'float',
  'int' => 'int',
  'unsigned char' => 'uint8',
  # 'char' => 'uint8',
}

RESERVED = %w[global map buffer]

def dereserve(name)
  if RESERVED.include? name
    "#{name}_"
  else
    name
  end
end

def dedef(type)
  DEDEF_MAP[type] || type
end

def process_dedefs(source)
  def_lines = source.lines.map(&:strip).select{|x| x =~ /^#define \w+\s+\d+/}
  defs = def_lines.map {|x|
    x, y = x.gsub(/\/\/.*/, "").gsub(/^#define/, "").split.map(&:strip)
    [x, y.to_i]
  }
  DEDEF_MAP.merge!(Hash[defs])
end

class String
  def blank_or_comment?
    self.strip.size == 0 || self.strip =~ /^\/\//
  end
end

def struct_regex(name)
  /struct #{name}(\s+[^\n]*)?\n\{(.*?)\};/ms
end

def anon_struct_regex
  /struct(.*?)\{(.*?)\}(.*?);/ms
end

def parse_struct(source, name, hints)
  source =~ struct_regex(name)
  content = $2
  subs = []
  # note that this won't work in general; luckily for us, the _mjVisual struct
  # only has anonymous struct fields and nothing else
  subprops = []
  hint_name = "#{name[1..-1].upcase}_POINTERS"
  struct_hint = hints[hint_name] || {}
  content.scan(anon_struct_regex) {
    subcontent = $2
    subname = $3
    subs << {
      props: subcontent.lines.map(&:strip).reject(&:blank_or_comment?).map{|x| parse_struct_line(source, x, struct_hint)},
      name: "ANON_#{subname.strip.gsub(/^_/,'').upcase}",
      source: source
    }
    subprops << {
      kind: :anonstruct, 
      type: "ANON_#{subname.strip.gsub(/^_/,'').upcase}",
      name: dereserve(subname.strip),
    }
  }
  rest = content.gsub(anon_struct_regex, '')
  rest = rest.lines.map(&:strip).reject(&:blank_or_comment?)
  parsed = rest.map {|x| parse_struct_line(source, x, struct_hint)}
  {
    props: subprops + parsed,
    name: dereserve(name),
    source: source,
    subs: subs,
  }
end

def parse_struct_line(source, line, hints)
  if line =~ /^(\w+)\s+(\w+)\s+(\w+);/
    {
      kind: :value,
      type: dedef($1 + " " + $2),
      name: $3
    }
  elsif line =~ /^(\w+)\s+(\w+);/
    {
      kind: :value,
      type: dedef($1),
      name: $2
    }
  elsif line =~ /^(\w+?)\*\s+(\w+?);/
    ret = {
      kind: :pointer,
      type: dedef($1),
      name: $2
    }
    # special case
    if ret[:name] == "buffer" && ret[:type] == "void"
      ret[:type] = "unsigned char"
    end
    if hints.size > 0
      match = hints.find{|x| x[1] == $2}
      if match 
        ret[:hint] = match[2..-1].map{|x| dedef(x)}
        # binding.pry
      end
    end
    unless ret[:hint]
      if line =~ /\/\/.*\((\w+)\s+(\w+)\)$/ # size hint
        ret[:hint] = [dedef($1)]
      elsif line =~ /\/\/.*\(([\w\*]+)\s+x\s+(\w+)\)$/ # size hint
        ret[:hint] = [dedef($1), dedef($2)]
      elsif line =~ /\/\/.*\((\w+)\)$/ # size hint
        ret[:hint] = [dedef($1)]
      end
    end
    ret
  elsif line =~ /(\w+)\s+(\w+)\[\s*(\w+)\s*\];/
    ret = {
      kind: :array,
      type: dedef($1),
      name: $2,
    }
    size = $3
    if size !~ /\d+/
      size = resolve_id_value(source, size)
    end
    ret[:size] = size
    ret
  elsif line =~ /(\w+)\s+(\w+)\[\s*(\w+)\s*\]\[\s*(\w+)\s*\];/
    ret = {
      kind: :double_array,
      type: dedef($1),
      name: $2,
    }
    size1 = $3
    size2 = $4
    if size1 !~ /\d+/
      size1 = resolve_id_value(source, size1)
    end
    if size2 !~ /\d+/
      size2 = resolve_id_value(source, size2)
    end
    ret[:size] = [size1, size2]
    ret
  else
    binding.pry
  end
end

def resolve_id_value(source, id)
  if DEDEF_MAP.include?(id)
    return DEDEF_MAP[id]
  end
  source =~ /enum\s+.*\{(.*?)\s+#{id}\s+(.*?)\}/ms
  if $1.nil?
    if source =~ /#define\s+#{id}\s+(\d+)/
      $1.to_i
    else
      binding.pry
    end
  else
    $1.lines.reject(&:blank_or_comment?).size
  end
end

def to_ctypes_type(prop)
  case prop[:kind]
  when :pointer
    CTYPES_PTR_MAP[prop[:type]] || \
      "POINTER(#{to_ctypes_type(prop.merge(kind: :value))})"
  when :anonstruct
    prop[:type]
  when :array
    "#{to_ctypes_type(prop.merge(kind: :value))} * #{prop[:size]}"
  when :value
    CTYPES_MAP[prop[:type]] || prop[:type].upcase
  when :double_array
    typ = "#{to_ctypes_type(prop.merge(kind: :value))} * #{prop[:size][-1]}"
    prop[:size].reverse[1..-1].each do |s|
      typ = "(#{typ}) * #{s}"
    end
    typ
  else
    puts prop
    raise :wtf
  end
end


def gen_ctypes_src(source, struct)
%Q{
class #{struct[:name].gsub(/^_/,'').upcase}(Structure):
    #{(struct[:subs] || []).map{|subs| gen_ctypes_src(source, subs).split("\n").join("\n    ")}.join("\n    ")}
    _fields_ = [
        #{struct[:props].map{|prop|
          "(\"#{prop[:name]}\", #{to_ctypes_type(prop)}),"
        }.join("\n        ")}
    ]
}
end



def gen_wrapper_src(source, struct)

  def to_size_factor(source, struct, hint_elem)
    hint_elem = hint_elem.to_s
    if hint_elem.to_s != hint_elem.to_s.downcase && hint_elem.to_s.size > 3
      if source =~ /#define\s+#{hint_elem}\s+(\d+)/
        $1.to_i
      else
        binding.pry
      end
    end
    if hint_elem =~ /\*/
      hint_elem.split("*").map{|x| to_size_factor(source, struct, x)}.join("*")
    elsif hint_elem =~ /^\d+$/
      hint_elem
    else
      if struct[:props].any?{|x| x[:name] == hint_elem}
        "self.#{hint_elem}"
      else
        "self._size_src.#{hint_elem}"
      end
    end
  end

%Q{
class #{struct[:name].gsub(/^_/,'').camelize}Wrapper(object):
    
    def __init__(self, wrapped, size_src=None):
        self._wrapped = wrapped
        self._size_src = size_src

    @property
    def ptr(self):
        return self._wrapped

    @property
    def obj(self):
        return self._wrapped.contents

    #{struct[:props].map{|prop|
      if ([:array, :pointer].include?(prop[:kind]) && NP_DTYPE_MAP.include?(prop[:type])) \
          || prop[:kind] == :pointer && prop[:name] == "buffer"
        dtype =
          if NP_DTYPE_MAP.include? prop[:type]
            NP_DTYPE_MAP[prop[:type]]
          elsif prop[:name] == "buffer"
            "uint8"
          else
            nil
          end

        if prop[:kind] == :pointer && dtype && prop[:hint]
          count = prop[:hint].map{|hint_elem| to_size_factor(source, struct, hint_elem)}.join("*")
          shape = "(" + prop[:hint].map{|hint_elem| to_size_factor(source, struct, hint_elem)}.join(", ") + ", )"
          ctype_type = to_ctypes_type(prop.merge(kind: :value))
        elsif prop[:kind] == :array
          count = prop[:size]
          shape = "(#{count}, )"
          ctype_type = to_ctypes_type(prop.merge(kind: :value))
        else
          $stderr.puts "ignoring #{prop}"
          next
        end
%Q{
@property
def #{prop[:name]}(self):
    arr = np.reshape(np.fromiter(self._wrapped.contents.#{prop[:name]}, dtype=np.#{dtype}, count=(#{count})), #{shape})
    arr.setflags(write=False)
    return arr

@#{prop[:name]}.setter
def #{prop[:name]}(self, value):
    val_ptr = np.array(value, dtype=np.float64).ctypes.data_as(POINTER(#{ctype_type}))
    memmove(self._wrapped.contents.#{prop[:name]}, val_ptr, #{count} * sizeof(#{ctype_type}))
}

      elsif prop[:kind] == :value || prop[:kind] == :anonstruct || prop[:kind] == :array
%Q{
@property
def #{prop[:name]}(self):
    return self._wrapped.contents.#{prop[:name]}

@#{prop[:name]}.setter
def #{prop[:name]}(self, value):
    self._wrapped.contents.#{prop[:name]} = value
}
      elsif prop[:kind] == :pointer && prop[:type] == 'char'
%Q{
@property
def #{prop[:name]}(self):
    return self._wrapped.contents.#{prop[:name]}
}
      elsif prop[:kind] == :pointer && prop[:type] == 'mjContact'
        # count = prop[:hint].map{|hint_elem| to_size_factor(source, struct, hint_elem)}.join("*")
        # binding.pry
      else
        # move on
        # binding.pry
      end
    }.compact.map{|propsrc| propsrc.split("\n").join("\n    ")}.join("\n    ")}
}
end


def parse_hints(hint_source)
  defs = []
  in_def = false
  cur_def = nil
  cur_def_name = nil
  hint_source.lines.each do |line|
    if line.strip =~ /^#define (\w+)/
      in_def = true
      cur_def_name = $1
      cur_def = [cur_def_name, []]
    elsif in_def
      filtered = line.strip.gsub(/^X\(/, "").gsub(/\)$/, "").gsub(/\)\s*\\$/, "")
      parts = filtered.split(",").map(&:strip)
      cur_def[1] << parts
      if line.strip[-1] != '\\'
        defs << cur_def
        in_def = false
      end
    end
  end
  Hash[defs]
end

source = open(ARGV.first, 'r').read
dim_hint_source = open(ARGV[1], 'r').read

process_dedefs(source)

hints = parse_hints(dim_hint_source)

source = source.gsub(/\r/, '')
source = source.gsub(/\/\*.*?\*\//, '')

puts %Q{
# AUTO GENERATED. DO NOT CHANGE!
from ctypes import *
import numpy as np
}

structs = %w[_mjContact _mjrRect _mjvCameraPose _mjrOption _mjrContext _mjvCamera _mjvOption _mjvGeom _mjvLight _mjvObjects _mjOption _mjVisual _mjStatistic _mjData _mjModel].map{|x| parse_struct(source, x, hints) }

structs.each {|s| puts gen_ctypes_src(source, s) }
structs.each {|s| puts gen_wrapper_src(source, s) }
