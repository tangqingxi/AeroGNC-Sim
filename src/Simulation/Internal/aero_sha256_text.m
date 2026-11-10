function digest = aero_sha256_text(text)
%AERO_SHA256_TEXT 对 UTF-8 文本计算 SHA-256（小写十六进制）
%
% 输入：char/string 文本。
% 输出：64 字符小写十六进制。
%
% 该函数只用于实验定义/身份哈希，不读取文件；文件字节哈希仍由调用方
% 或 WTR 的 wtr_sha256 处理。依赖 MATLAB 自带 JVM。

if isstring(text)
    text = char(text);
end
if ~ischar(text)
    error('aero:sha:text', '输入必须是 char 或 string');
end
bytes = unicode2native(text, 'UTF-8');
md = java.security.MessageDigest.getInstance('SHA-256');
md.update(bytes);
raw = typecast(md.digest(), 'uint8');
digest = lower(sprintf('%02x', raw));

end
