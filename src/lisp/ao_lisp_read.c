/*
 * Copyright © 2016 Keith Packard <keithp@keithp.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include "ao_lisp.h"
#include "ao_lisp_read.h"

static const uint16_t	lex_classes[128] = {
	IGNORE,		/* ^@ */
	IGNORE,		/* ^A */
	IGNORE,		/* ^B */
	IGNORE,		/* ^C */
	IGNORE,		/* ^D */
	IGNORE,		/* ^E */
	IGNORE,		/* ^F */
	IGNORE,		/* ^G */
	IGNORE,		/* ^H */
	WHITE,		/* ^I */
	WHITE,		/* ^J */
	WHITE,		/* ^K */
	WHITE,		/* ^L */
	WHITE,		/* ^M */
	IGNORE,		/* ^N */
	IGNORE,		/* ^O */
	IGNORE,		/* ^P */
	IGNORE,		/* ^Q */
	IGNORE,		/* ^R */
	IGNORE,		/* ^S */
	IGNORE,		/* ^T */
	IGNORE,		/* ^U */
	IGNORE,		/* ^V */
	IGNORE,		/* ^W */
	IGNORE,		/* ^X */
	IGNORE,		/* ^Y */
	IGNORE,		/* ^Z */
	IGNORE,		/* ^[ */
	IGNORE,		/* ^\ */
	IGNORE,		/* ^] */
	IGNORE,		/* ^^ */
	IGNORE,		/* ^_ */
	PRINTABLE|WHITE,	/*    */
 	PRINTABLE,		/* ! */
 	PRINTABLE|STRINGC,	/* " */
 	PRINTABLE|COMMENT,	/* # */
 	PRINTABLE,		/* $ */
 	PRINTABLE,		/* % */
 	PRINTABLE,		/* & */
 	PRINTABLE|QUOTEC,	/* ' */
 	PRINTABLE|BRA,		/* ( */
 	PRINTABLE|KET,		/* ) */
 	PRINTABLE,		/* * */
 	PRINTABLE|SIGN,		/* + */
 	PRINTABLE,		/* , */
 	PRINTABLE|SIGN,		/* - */
 	PRINTABLE,		/* . */
 	PRINTABLE,		/* / */
 	PRINTABLE|DIGIT,	/* 0 */
 	PRINTABLE|DIGIT,	/* 1 */
 	PRINTABLE|DIGIT,	/* 2 */
 	PRINTABLE|DIGIT,	/* 3 */
 	PRINTABLE|DIGIT,	/* 4 */
 	PRINTABLE|DIGIT,	/* 5 */
 	PRINTABLE|DIGIT,	/* 6 */
 	PRINTABLE|DIGIT,	/* 7 */
 	PRINTABLE|DIGIT,	/* 8 */
 	PRINTABLE|DIGIT,	/* 9 */
 	PRINTABLE,		/* : */
 	PRINTABLE|COMMENT,	/* ; */
 	PRINTABLE,		/* < */
 	PRINTABLE,		/* = */
 	PRINTABLE,		/* > */
 	PRINTABLE,		/* ? */
  	PRINTABLE,		/*  @ */
	PRINTABLE,		/*  A */
	PRINTABLE,		/*  B */
	PRINTABLE,		/*  C */
	PRINTABLE,		/*  D */
	PRINTABLE,		/*  E */
	PRINTABLE,		/*  F */
	PRINTABLE,		/*  G */
	PRINTABLE,		/*  H */
	PRINTABLE,		/*  I */
	PRINTABLE,		/*  J */
	PRINTABLE,		/*  K */
	PRINTABLE,		/*  L */
	PRINTABLE,		/*  M */
	PRINTABLE,		/*  N */
	PRINTABLE,		/*  O */
	PRINTABLE,		/*  P */
	PRINTABLE,		/*  Q */
	PRINTABLE,		/*  R */
	PRINTABLE,		/*  S */
	PRINTABLE,		/*  T */
	PRINTABLE,		/*  U */
	PRINTABLE,		/*  V */
	PRINTABLE,		/*  W */
	PRINTABLE,		/*  X */
	PRINTABLE,		/*  Y */
	PRINTABLE,		/*  Z */
	PRINTABLE,		/*  [ */
	PRINTABLE|BACKSLASH,	/*  \ */
	PRINTABLE,		/*  ] */
	PRINTABLE,		/*  ^ */
	PRINTABLE,		/*  _ */
  	PRINTABLE,		/*  ` */
	PRINTABLE,		/*  a */
	PRINTABLE,		/*  b */
	PRINTABLE,		/*  c */
	PRINTABLE,		/*  d */
	PRINTABLE,		/*  e */
	PRINTABLE,		/*  f */
	PRINTABLE,		/*  g */
	PRINTABLE,		/*  h */
	PRINTABLE,		/*  i */
	PRINTABLE,		/*  j */
	PRINTABLE,		/*  k */
	PRINTABLE,		/*  l */
	PRINTABLE,		/*  m */
	PRINTABLE,		/*  n */
	PRINTABLE,		/*  o */
	PRINTABLE,		/*  p */
	PRINTABLE,		/*  q */
	PRINTABLE,		/*  r */
	PRINTABLE,		/*  s */
	PRINTABLE,		/*  t */
	PRINTABLE,		/*  u */
	PRINTABLE,		/*  v */
	PRINTABLE,		/*  w */
	PRINTABLE,		/*  x */
	PRINTABLE,		/*  y */
	PRINTABLE,		/*  z */
	PRINTABLE,		/*  { */
	PRINTABLE|VBAR,		/*  | */
	PRINTABLE,		/*  } */
	PRINTABLE|TWIDDLE,	/*  ~ */
	IGNORE,			/*  ^? */
};

static int lex_unget_c;

static inline int
lex_get()
{
	int	c;
	if (lex_unget_c) {
		c = lex_unget_c;
		lex_unget_c = 0;
	} else
		c = getchar();
	return c;
}

static inline void
lex_unget(int c)
{
	if (c != EOF)
		lex_unget_c = c;
}

static int
lex_quoted (void)
{
	int	c;
	int	v;
	int	count;

	c = lex_get();
//	if (jumping)
//		return nil;
	if (c == EOF)
		return EOF;
	c &= 0x7f;
 	switch (c) {
	case 'n':
		return '\n';
	case 'f':
		return '\f';
	case 'b':
		return '\b';
	case 'r':
		return '\r';
	case 'v':
		return '\v';
	case 't':
		return '\t';
	case '0':
	case '1':
	case '2':
	case '3':
	case '4':
	case '5':
	case '6':
	case '7':
		v = c - '0';
		count = 1;
		while (count <= 3) {
			c = lex_get();
//			if (jumping)
//				return nil;
			if (c == EOF)
				return EOF;
			c &= 0x7f;
			if (c < '0' || '7' < c) {
				lex_unget(c);
				break;
			}
			v = (v << 3) + c - '0';
			++count;
		}
		return v;
	default:
		return c;
	}
}

static uint16_t	lex_class;

static int
lexc(void)
{
	int	c;
	do {
		c = lex_get();
		if (c == EOF) {
			lex_class = ENDOFFILE;
			c = 0;
		} else {
			c &= 0x7f;
			lex_class = lex_classes[c];
			if (lex_class & BACKSLASH) {
				c = lex_quoted();
				if (c == EOF)
					lex_class = ENDOFFILE;
				else
					lex_class = PRINTABLE;
			}
		}
	} while (lex_class & IGNORE);
	return c;
}

#define AO_LISP_TOKEN_MAX	32

static char	token_string[AO_LISP_TOKEN_MAX];
static int	token_int;
static int	token_len;

static inline void add_token(int c) {
	if (c && token_len < AO_LISP_TOKEN_MAX - 1)
		token_string[token_len++] = c;
}

static inline void end_token(void) {
	token_string[token_len] = '\0';
}

static int
lex(void)
{
	int	c;

	token_len = 0;
	for (;;) {
		c = lexc();
		if (lex_class & ENDOFFILE)
			return AO_LISP_NIL;

//		if (jumping)
//			return nil;
		if (lex_class & WHITE)
			continue;

		if (lex_class & (BRA|KET|QUOTEC)) {
			add_token(c);
			end_token();
			switch (c) {
			case '(':
				return OPEN;
			case ')':
				return CLOSE;
			case '\'':
				return QUOTE;
			}
		}
		if (lex_class & TWIDDLE) {
			token_int = lexc();
			return NUM;
		}
		if (lex_class & STRINGC) {
			for (;;) {
				c = lexc();
//				if (jumping)
//					return nil;
				if (lex_class & (STRINGC|ENDOFFILE)) {
					end_token();
					return STRING;
				}
				add_token(c);
			}
		}
		if (lex_class & PRINTABLE) {
			int	isnum;
			int	hasdigit;
			int	isneg;

			isnum = 1;
			hasdigit = 0;
			token_int = 0;
			isneg = 0;
			for (;;) {
				if (!(lex_class & NUMBER)) {
					isnum = 0;
				} else {
 					if (token_len != 0 &&
					    (lex_class & SIGN))
					{
						isnum = 0;
					}
					if (c == '-')
						isneg = 1;
					if (lex_class & DIGIT) {
						hasdigit = 1;
						if (isnum)
							token_int = token_int * 10 + c - '0';
					}
				}
				add_token (c);
				c = lexc ();
//				if (jumping)
//					return nil;
				if (lex_class & (NOTNAME)) {
//					if (lex_class & ENDOFFILE)
//						clearerr (f);
					lex_unget(c);
					end_token ();
					if (isnum && hasdigit) {
						if (isneg)
							token_int = -token_int;
						return NUM;
					}
					return NAME;
				}
			}

		}
	}
}

static int parse_token;
static uint8_t			been_here;
static struct ao_lisp_cons	*read_cons;
static struct ao_lisp_cons	*read_cons_tail;
static struct ao_lisp_cons	*read_stack;

static ao_lisp_poly
read_item(void)
{
	struct ao_lisp_atom	*atom;
	char			*string;
	int			cons;
	ao_lisp_poly		v;

	if (!been_here) {
		ao_lisp_root_add(&ao_lisp_cons_type, &read_cons);
		ao_lisp_root_add(&ao_lisp_cons_type, &read_cons_tail);
		ao_lisp_root_add(&ao_lisp_cons_type, &read_stack);
	}

	cons = 0;
	read_cons = read_cons_tail = read_stack = 0;
	for (;;) {
		while (parse_token == OPEN) {
			if (cons++)
				read_stack = ao_lisp_cons(ao_lisp_cons_poly(read_cons), read_stack);
			read_cons = NULL;
			read_cons_tail = NULL;
			parse_token = lex();
		}

		switch (parse_token) {
		case ENDOFFILE:
		default:
			v = AO_LISP_NIL;
			break;
		case NAME:
			atom = ao_lisp_atom_intern(token_string);
			if (atom)
				v = ao_lisp_atom_poly(atom);
			else
				v = AO_LISP_NIL;
			break;
		case NUM:
			v = ao_lisp_int_poly(token_int);
			break;
		case STRING:
			string = ao_lisp_string_copy(token_string);
			if (string)
				v = ao_lisp_string_poly(string);
			else
				v = AO_LISP_NIL;
			break;
		case CLOSE:
			if (cons)
				v = ao_lisp_cons_poly(read_cons);
			else
				v = AO_LISP_NIL;
			if (--cons) {
				read_cons = ao_lisp_poly_cons(read_stack->car);
				read_stack = read_stack->cdr;
				for (read_cons_tail = read_cons;
				     read_cons_tail && read_cons_tail->cdr;
				     read_cons_tail = read_cons_tail->cdr)
					;
			}
			break;
		}

		if (!cons)
			break;

		struct ao_lisp_cons	*read = ao_lisp_cons(v, NULL);
		if (read_cons_tail)
			read_cons_tail->cdr = read;
		else
			read_cons = read;
		read_cons_tail = read;

		parse_token = lex();
	}
	return v;
}

ao_lisp_poly
ao_lisp_read(void)
{
	parse_token = lex();
	return read_item();
}
