//for the WinUSB project
//see also wugadget.c

//derived from: https://blog.soutade.fr/post/2016/07/create-your-own-usb-gadget-with-gadgetfs.html

// MIT License

// Copyright (c) 2016 Grégory Soutadé
// 2020 WinUSB feature added by beb @ stepover.de and rundekugel @ github.com

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

/* From usbstring.[ch] */
struct usb_string {
    __u8            id;
    const char        *s;
};

struct usb_gadget_strings {
    __u16            language;    /* 0x0409 for en-us */
    struct usb_string    *strings;
};


static inline void put_unaligned_le16(__u16 val, __u16 *cp)
{
    __u8    *p = (void *)cp;

    *p++ = (__u8) val;
    *p++ = (__u8) (val >> 8);
}

static int utf8_to_utf16le(const char *s, __u16 *cp, unsigned len)
{
    int    count = 0;
    __u8    c;
    __u16    uchar;

    /* this insists on correct encodings, though not minimal ones.
     * BUT it currently rejects legit 4-byte UTF-8 code points,
     * which need surrogate pairs.  (Unicode 3.1 can use them.)
     */
    while (len != 0 && (c = (__u8) *s++) != 0) {
        if (c & 0x80) {
            // 2-byte sequence:
            // 00000yyyyyxxxxxx = 110yyyyy 10xxxxxx
            if ((c & 0xe0) == 0xc0) {
                uchar = (c & 0x1f) << 6;

                c = (__u8) *s++;
                if ((c & 0xc0) != 0xc0)
                    goto fail;
                c &= 0x3f;
                uchar |= c;

            // 3-byte sequence (most CJKV characters):
            // zzzzyyyyyyxxxxxx = 1110zzzz 10yyyyyy 10xxxxxx
            } else if ((c & 0xf0) == 0xe0) {
                uchar = (c & 0x0f) << 12;

                c = (__u8) *s++;
                if ((c & 0xc0) != 0xc0)
                    goto fail;
                c &= 0x3f;
                uchar |= c << 6;

                c = (__u8) *s++;
                if ((c & 0xc0) != 0xc0)
                    goto fail;
                c &= 0x3f;
                uchar |= c;

                /* no bogus surrogates */
                if (0xd800 <= uchar && uchar <= 0xdfff)
                    goto fail;

            // 4-byte sequence (surrogate pairs, currently rare):
            // 11101110wwwwzzzzyy + 110111yyyyxxxxxx
            //     = 11110uuu 10uuzzzz 10yyyyyy 10xxxxxx
            // (uuuuu = wwww + 1)
            // FIXME accept the surrogate code points (only)

            } else
                goto fail;
        } else
            uchar = c;
        put_unaligned_le16 (uchar, cp++);
        count++;
        len--;
    }
    return count;
fail:
    return -1;
}

int
usb_gadget_get_string (struct usb_gadget_strings *table, int id, __u8 *buf)
{
    struct usb_string    *s;
    int            len;

    /* descriptor 0 has the language id */
    if (id == 0) {
        buf [0] = 4;
        buf [1] = USB_DT_STRING;
        buf [2] = (__u8) table->language;
        buf [3] = (__u8) (table->language >> 8);
        return 4;
    }
    for (s = table->strings; s && s->s; s++)
        if (s->id == id)
            break;

    /* unrecognized: stall. */
    if (!s || !s->s)
        return -EINVAL;

    /* string descriptors have length, tag, then UTF16-LE text */
    len = strlen (s->s);
    if (len > 126)
        len = 126;
    memset (buf + 2, 0, 2 * len);    /* zero all the bytes */
    len = utf8_to_utf16le(s->s, (__u16 *)&buf[2], len);
    if (len < 0)
        return -EINVAL;
    buf [0] = (len + 1) * 2;
    buf [1] = USB_DT_STRING;
    return buf [0];
}

//eof
