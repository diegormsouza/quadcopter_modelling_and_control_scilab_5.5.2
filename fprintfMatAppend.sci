function b = fprintfMatAppend(filename, data, comments)
  if isfile(filename) then
    txt = mgetl(filename);
  else
    txt = [];
  end
  fprintfMat(TMPDIR + "/Mat", data,"%5.7f",comments);
  txt = [txt;mgetl(TMPDIR + "/Mat")];
  b = mputl(txt, filename);
endfunction
