import unittest

from c_extract import extract_block


class ExtractBlockTests(unittest.TestCase):
    def test_literals_comments_and_unchanged_source(self):
        definition = 'static int f(void) { char c = \'}\'; const char *s = "\\\"{"; /* } */\n // {\n return c; }'
        source = '/* int f(void) {} */\n' + definition + '\nint other(void) { return 0; }'
        self.assertEqual(extract_block(source, r'^static int f\(void\)\s*\{'), definition)

    def test_struct_semicolon_with_comment(self):
        source = 'struct foo { int a; } /* trivia */;'
        self.assertEqual(extract_block(source, r'^struct foo\s*\{', semicolon=True), source)

    def test_missing_duplicate_unclosed_and_bad_selector(self):
        cases = [
            ('int x;', r'^int f\(void\)\s*\{'),
            ('int f(void) {}\nint f(void) {}', r'^int f\(void\)\s*\{'),
            ('int f(void) {', r'^int f\(void\)\s*\{'),
            ('int f(void) {}', r'^int f'),
            ('int f(void) { /*', r'^int f\(void\)\s*\{'),
            ('int f(void) { "unterminated', r'^int f\(void\)\s*\{'),
        ]
        for source, selector in cases:
            with self.subTest(source=source), self.assertRaises(ValueError):
                extract_block(source, selector)
        with self.assertRaises(ValueError):
            extract_block('struct foo { int a; }', r'^struct foo\s*\{', semicolon=True)


if __name__ == '__main__':
    unittest.main()
