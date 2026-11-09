function s = wtr_sha256(data)
%WTR_SHA256 SHA-256（小写十六进制）
%
% 输入：data 为 uint8 字节数组或文件名（char/string，读取文件字节）。
% 输出：64 字符十六进制字符串。
% 依赖 Java 标准库（MATLAB R2025b 自带 JVM），不引入第三方包。

if ischar(data) || (isstring(data) && isscalar(data))
    file = char(data);
    fid = fopen(file, 'r');
    if fid < 0
        error('wtr:sha:file', '无法读取文件：%s', file);
    end
    bytes = fread(fid, Inf, '*uint8');
    fclose(fid);
else
    bytes = uint8(data(:));
end

md = java.security.MessageDigest.getInstance('SHA-256');
md.update(bytes);
digest = typecast(md.digest(), 'uint8');
s = lower(sprintf('%02x', digest));

end
