#!ruby -Ku
if ARGV.size != 1 then
    puts "Argment Error!"
    exit
end

while str = STDIN.gets
    if /^.*\.data\s+[\dabcdef]+\s+[\dabcdef]+\s+([\dabcdef]+)\s+.*/ =~ str
        command = 'v850-elf-objcopy  -R .bss ' + ARGV[0]
        puts command
        system(command)        
        command = 'v850-elf-objcopy  --change-section-vma .data=0x' + $1 + ' ' + ARGV[0]
        puts command
        system(command)
        exit(0)
    end
end
