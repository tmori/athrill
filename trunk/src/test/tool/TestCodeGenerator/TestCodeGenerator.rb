#! ruby -Ks
require 'erb'
require 'csv'
require 'fileutils'
require 'TestObjectCreator.rb'

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

def main(test_item_file, csv_file, tpath)
  creator = TestObjectCreator.new(test_item_file)
  spec = creator.load()
  
  parser = TemplateFileParser.new(csv_file)
  parser.parse

  #generate for each template
  parser.get.each { |entry|
    src = render(tpath + "/" + entry.template)
    open(tpath + "/" + entry.output, "w") { |f|
      f.write(src)
      puts "### Created: " + entry.output
    }
  }
end

if ARGV.length != 3
  printf("Usage: TestCodeGenerator.rb test_item_file template.csv tpath")
  exit(1)
end

test_item_file=ARGV[0]
csv_file=ARGV[1]
tpath=ARGV[2]

begin
  main(test_item_file, csv_file, tpath)
rescue => e
  p e.message
  exit(1)
end

exit(0)
