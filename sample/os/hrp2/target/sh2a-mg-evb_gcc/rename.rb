#!/usr/bin/ruby
#

while line = ARGF.gets
    if /^[^ ]* [A-Z] / =~ line
    elsif /^[ ]*[U] / =~ line
    else
        puts line.sub(/^[^ ]* [^ ]* /, '')
    end
end

