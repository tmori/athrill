#! ruby -Ks
require 'erb'
require 'csv'
require 'fileutils'
require 'yaml'
require File.dirname(__FILE__) + '/parser/types/TypeBuilder'

class TemplateFileMap
    attr_accessor :output
    attr_accessor :template

    def initialize(line)
  self.output   = line[0].strip
  self.template   = line[1].strip
    end
end

class TemplateFileParser
  def initialize(file)
    @filepath = file
    @list = Array.new()
  end

  def parse()
    CSV.foreach(@filepath) do |line|
      @list[@list.length] = TemplateFileMap.new(line)
    end
  end

  def get()
    return @list
  end
end

def render path
  src = ERB.new(File.read(path), nil, '-').result(binding)
end

def config_parse(config_file_folderpath)
  config = YAML.load_file(config_file_folderpath + "/PrimitiveType.yaml")
  TypeBuilder.buildPrimitiveType(config)
  
  config = YAML.load_file(config_file_folderpath + "/DataElementType.yaml")
  TypeBuilder.buildDataElementType(config)
  
  config = YAML.load_file(config_file_folderpath + "/BusElementType.yaml")
  TypeBuilder.buildBusElementType(config)
  
  config = YAML.load_file(config_file_folderpath + "/BusType.yaml")
  TypeBuilder.buildBusType(config)
end

def main(config_path, csv_file, root_path)
  config_parse(config_path)
  
  parser = TemplateFileParser.new(csv_file)
  parser.parse

  #generate for each template
  parser.get.each { |entry|
    src = render(root_path + "/template/" + entry.template)
    open(root_path + "/generated/" + entry.output, "w") { |f|
      f.write(src)
      puts "### Created: " + entry.output
    }
  }
end

if ARGV.length != 3
  printf("Usage: AcommCodeGenerator.rb config_path template.csv root_path\n")
  exit(1)
end

path=ARGV[0]
csv_file=ARGV[1]
root_path=ARGV[2]

begin
  main(path, csv_file, root_path)
rescue => e
  p e.message
  exit(1)
end

exit(0)
