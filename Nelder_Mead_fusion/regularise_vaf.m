function outp = regularise_vaf(inp)

if inp < 1
    outp = inp;
else
    outp = 1/inp;
end

end