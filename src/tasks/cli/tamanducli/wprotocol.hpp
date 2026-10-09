#ifndef WPROTOCOL_HPP
#define WPROTOCOL_HPP

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

/**
 * @namespace wire
 * @brief Helpers do protocolo de comunicação em texto (wire protocol).
 *
 * Formato dos comandos: `nome(modo,req_ou_resp,...args);`
 * Ver docs/specs.md e docs/wire_protocol_firmware_implementation.md.
 */
namespace wire {

/** @brief Número máximo de argumentos por comando. */
constexpr int kMaxArgs = 24;

/** @brief Capacidade máxima (bytes) de cada argumento parseado. */
constexpr size_t kArgCap = 160;

/** @brief Capacidade máxima (bytes) do nome do comando. */
constexpr size_t kNameCap = 64;

/**
 * @brief Orçamento de bytes para empacotar header + bodies em uma mensagem BLE.
 *
 * Alinhar com o limite do transporte (ex.: 256 bytes no NUS).
 */
constexpr size_t kPackBudget = 240;

/**
 * @brief Comando wire parseado a partir de um segmento `nome(modo,role,...);`.
 */
struct Command {
  char name[kNameCap]; /**< Nome do comando (ex.: `param_list`). */
  char mode;           /**< `s` single, `h` header de lista, `b` body de lista. */
  char role;           /**< `r` requisição, `s` resposta. */
  int  bodyIndex;      /**< Índice da linha quando `mode == 'b'`; -1 caso contrário. */
  int  argc;           /**< Total de tokens em `argv` (inclui modo e role). */
  char argv[kMaxArgs][kArgCap]; /**< Argumentos brutos já sem aspas/escapes. */
};

/**
 * @brief Campos do header de lista `nome(h,role,T,C,B,j);`.
 */
struct ListHeader {
  int T; /**< Total de linhas da lista. */
  int C; /**< Linhas (bodies) nesta mensagem. */
  int B; /**< Total de mensagens que compõem a lista. */
  int j; /**< Índice desta mensagem (0 .. B-1). */
};

/**
 * @brief Compara duas strings C por igualdade.
 * @param a Primeira string.
 * @param b Segunda string.
 * @return `true` se forem idênticas.
 */
inline bool nameEq(const char *a, const char *b) { return strcmp(a, b) == 0; }

/**
 * @brief Retorna o intervalo `[a, b)` sem espaços nas extremidades de `s[0..n)`.
 * @param s Buffer de entrada.
 * @param n Comprimento de `s`.
 * @param[out] a Índice inicial (inclusive) após trim.
 * @param[out] b Índice final (exclusive) após trim.
 */
inline void trimView(const char *s, size_t n, size_t &a, size_t &b) {
  a = 0;
  b = n;
  while(a < b && isspace(static_cast<unsigned char>(s[a]))) {
    a++;
  }
  while(b > a && isspace(static_cast<unsigned char>(s[b - 1]))) {
    b--;
  }
}

/**
 * @brief Divide `s[0..n)` em segmentos pelo delimitador de nível superior.
 *
 * Não divide dentro de parênteses aninhados nem dentro de strings `"..."`.
 *
 * @param s Buffer de entrada.
 * @param n Comprimento de `s`.
 * @param delim Delimitador (ex.: `';'` ou `','`).
 * @param[out] ranges Pares `(início, fim)` de cada segmento não vazio.
 * @return Sempre `true` (reservado para extensão).
 */
inline bool splitTopLevel(const char *s, size_t n, char delim,
                          std::vector<std::pair<size_t, size_t>> &ranges) {
  ranges.clear();
  int    depth = 0;
  bool   inStr = false;
  size_t start = 0;
  for(size_t i = 0; i <= n; i++) {
    if(i == n || (s[i] == delim && !inStr && depth == 0)) {
      size_t a, b;
      trimView(s + start, i - start, a, b);
      if(a < b) {
        ranges.push_back({start + a, start + b});
      }
      start = i + 1;
      continue;
    }
    char c = s[i];
    if(inStr) {
      if(c == '\\' && i + 1 < n) {
        i++;
        continue;
      }
      if(c == '"') {
        inStr = false;
      }
    } else {
      if(c == '"') {
        inStr = true;
      } else if(c == '(') {
        depth++;
      } else if(c == ')' && depth > 0) {
        depth--;
      }
    }
  }
  return true;
}

/**
 * @brief Localiza o `)` que fecha o `(` em `openIdx`, respeitando strings e aninhamento.
 * @param s Buffer de entrada.
 * @param openIdx Índice do `(` de abertura.
 * @param n Comprimento de `s`.
 * @param[out] closeIdx Índice do `)` correspondente.
 * @return `true` se o parêntese de fechamento foi encontrado.
 */
inline bool findMatchingClose(const char *s, size_t openIdx, size_t n,
                              size_t &closeIdx) {
  int  depth = 0;
  bool inStr = false;
  for(size_t i = openIdx; i < n; i++) {
    char c = s[i];
    if(inStr) {
      if(c == '\\' && i + 1 < n) {
        i++;
        continue;
      }
      if(c == '"') {
        inStr = false;
      }
    } else {
      if(c == '"') {
        inStr = true;
      } else if(c == '(') {
        depth++;
      } else if(c == ')') {
        depth--;
        if(depth == 0) {
          closeIdx = i;
          return true;
        }
      }
    }
  }
  return false;
}

/**
 * @brief Remove aspas e unescape de um token wire para `out`.
 *
 * Tokens sem aspas são copiados após trim. Sequências `\\` e `\"` são expandidas.
 *
 * @param src Token de entrada.
 * @param len Comprimento de `src`.
 * @param[out] out Buffer de saída.
 * @param outCap Capacidade de `out`.
 * @return `false` se `outCap == 0` ou o valor não couber em `out`.
 */
inline bool unquoteField(const char *src, size_t len, char *out,
                         size_t outCap) {
  if(outCap == 0) {
    return false;
  }
  size_t a, b;
  trimView(src, len, a, b);
  if(a >= b) {
    out[0] = '\0';
    return true;
  }
  src += a;
  len = b - a;
  if(len >= 2 && src[0] == '"' && src[len - 1] == '"') {
    size_t w = 0;
    for(size_t i = 1; i + 1 < len && w + 1 < outCap; i++) {
      if(src[i] == '\\' && i + 1 < len - 1) {
        char nxt = src[i + 1];
        if(nxt == '\\' || nxt == '"') {
          out[w++] = nxt;
          i++;
          continue;
        }
      }
      out[w++] = src[i];
    }
    out[w] = '\0';
    return true;
  }
  if(len >= outCap) {
    return false;
  }
  memcpy(out, src, len);
  out[len] = '\0';
  return true;
}

/**
 * @brief Copia um token para `cmd.argv[idx]` após unquote.
 * @param cmd Comando de destino.
 * @param idx Índice em `argv`.
 * @param src Token bruto.
 * @param len Comprimento de `src`.
 * @return `false` se `idx` for inválido ou o token não couber.
 */
inline bool copyTokenToArgv(Command &cmd, int idx, const char *src,
                            size_t len) {
  if(idx < 0 || idx >= kMaxArgs) {
    return false;
  }
  return unquoteField(src, len, cmd.argv[idx], kArgCap);
}

/**
 * @brief Verifica se `s` é um inteiro decimal opcionalmente negativo.
 * @param s String a testar.
 * @return `true` se for inteiro puro (ex.: `42`, `-7`).
 */
inline bool tokenIsPlainInteger(const char *s) {
  if(s == nullptr || s[0] == '\0') {
    return false;
  }
  const char *p = s;
  if(*p == '-') {
    p++;
  }
  if(*p == '\0') {
    return false;
  }
  for(; *p; p++) {
    if(!isdigit(static_cast<unsigned char>(*p))) {
      return false;
    }
  }
  return true;
}

/**
 * @brief Verifica se `s` é um identificador C válido (`[a-zA-Z_][a-zA-Z0-9_]*`).
 * @param s String a testar.
 * @return `true` se for identificador puro.
 */
inline bool tokenIsPlainIdentifier(const char *s) {
  if(s == nullptr || s[0] == '\0') {
    return false;
  }
  if(!isalpha(static_cast<unsigned char>(*s)) && *s != '_') {
    return false;
  }
  for(const char *p = s + 1; *p; p++) {
    if(!isalnum(static_cast<unsigned char>(*p)) && *p != '_') {
      return false;
    }
  }
  return true;
}

/**
 * @brief Indica se o token precisa de aspas na serialização wire.
 * @param s Valor a emitir.
 * @return `true` se não for inteiro nem identificador puro.
 */
inline bool tokenNeedsQuotes(const char *s) {
  return !tokenIsPlainInteger(s) && !tokenIsPlainIdentifier(s);
}

/**
 * @brief Acrescenta um token wire em `out`, com aspas/escapes se necessário.
 * @param[out] out Buffer de saída.
 * @param outCap Capacidade de `out`.
 * @param[in,out] pos Posição atual de escrita em `out`.
 * @param s Valor do token.
 * @return `false` se não couber em `out`.
 */
inline bool appendWireToken(char *out, size_t outCap, size_t &pos,
                            const char *s) {
  auto appendRaw = [&](const char *r, size_t n) -> bool {
    if(pos + n >= outCap) {
      return false;
    }
    memcpy(out + pos, r, n);
    pos += n;
    out[pos] = '\0';
    return true;
  };

  if(!tokenNeedsQuotes(s)) {
    size_t n = strlen(s);
    return appendRaw(s, n);
  }
  if(!appendRaw("\"", 1)) {
    return false;
  }
  for(const char *p = s; *p; p++) {
    if(*p == '\\' || *p == '"') {
      if(!appendRaw("\\", 1) || !appendRaw(p, 1)) {
        return false;
      }
    } else {
      if(!appendRaw(p, 1)) {
        return false;
      }
    }
  }
  return appendRaw("\"", 1);
}

/**
 * @brief Acrescenta uma vírgula ao buffer wire em construção.
 * @param[out] buf Buffer de saída.
 * @param cap Capacidade de `buf`.
 * @param[in,out] pos Posição atual de escrita.
 * @return `false` se não couber.
 */
inline bool appendComma(char *buf, size_t cap, size_t &pos) {
  if(pos + 2 >= cap) {
    return false;
  }
  buf[pos++] = ',';
  buf[pos]   = '\0';
  return true;
}

/**
 * @brief Converte uma string em `float`.
 * @param s Texto decimal (ex.: `"3.14"`).
 * @param[out] out Valor parseado.
 * @return `false` se `s` for nulo, vazio ou inválido.
 */
inline bool parseFloat(const char *s, float &out) {
  if(s == nullptr || s[0] == '\0') {
    return false;
  }
  char *end = nullptr;
  out       = strtof(s, &end);
  return end != s && end != nullptr && *end == '\0';
}

/**
 * @brief Converte uma string em `int` decimal.
 * @param s Texto inteiro (ex.: `"-42"`).
 * @param[out] out Valor parseado.
 * @return `false` se `s` for nulo, vazio ou inválido.
 */
inline bool parseInt(const char *s, int &out) {
  if(s == nullptr || s[0] == '\0') {
    return false;
  }
  char *end = nullptr;
  long  v   = strtol(s, &end, 10);
  if(end == s || end == nullptr || *end != '\0') {
    return false;
  }
  out = static_cast<int>(v);
  return true;
}

/**
 * @brief Faz parse de um segmento `nome(modo,role,...);` em `out`.
 * @param seg Início do segmento (sem `;` final obrigatório).
 * @param segLen Comprimento de `seg`.
 * @param[out] out Comando parseado.
 * @return `false` se o formato for inválido ou exceder limites internos.
 */
inline bool parseSegment(const char *seg, size_t segLen, Command &out) {
  memset(&out, 0, sizeof(out));
  out.bodyIndex = -1;
  char lineBuf[512];
  if(segLen >= sizeof(lineBuf)) {
    return false;
  }
  memcpy(lineBuf, seg, segLen);
  lineBuf[segLen] = '\0';

  size_t n       = strlen(lineBuf);
  size_t openIdx = n;
  for(size_t i = 0; i < n; i++) {
    if(lineBuf[i] == '(') {
      openIdx = i;
      break;
    }
  }
  if(openIdx >= n) {
    return false;
  }
  size_t nameA, nameB;
  trimView(lineBuf, openIdx, nameA, nameB);
  if(nameA >= nameB || nameB - nameA >= kNameCap) {
    return false;
  }
  memcpy(out.name, lineBuf + nameA, nameB - nameA);
  out.name[nameB - nameA] = '\0';

  size_t closeIdx = 0;
  if(!findMatchingClose(lineBuf, openIdx, n, closeIdx)) {
    return false;
  }
  const char *inner    = lineBuf + openIdx + 1;
  size_t      innerLen = closeIdx - openIdx - 1;

  std::vector<std::pair<size_t, size_t>> commaRanges;
  splitTopLevel(inner, innerLen, ',', commaRanges);
  if(commaRanges.size() < 2) {
    return false;
  }
  out.argc = static_cast<int>(commaRanges.size());
  if(out.argc > kMaxArgs) {
    return false;
  }
  for(int i = 0; i < out.argc; i++) {
    auto pr = commaRanges[static_cast<size_t>(i)];
    if(!copyTokenToArgv(out, i, inner + pr.first, pr.second - pr.first)) {
      return false;
    }
  }
  if(strlen(out.argv[0]) != 1 || strlen(out.argv[1]) != 1) {
    return false;
  }
  out.mode = out.argv[0][0];
  out.role = out.argv[1][0];
  if(out.mode == 'b') {
    if(out.argc < 3) {
      return false;
    }
    out.bodyIndex = atoi(out.argv[2]);
  }
  return true;
}

/**
 * @brief Faz parse de uma mensagem completa com um ou mais comandos separados por `;`.
 * @param msg Mensagem wire (ex.: `help(s,r);param_list(s,r);`).
 * @param[out] out Lista de comandos parseados.
 * @return `false` se `msg` for nulo ou algum segmento for inválido.
 */
inline bool parseMessage(const char *msg, std::vector<Command> &out) {
  out.clear();
  if(msg == nullptr) {
    return false;
  }
  std::vector<std::pair<size_t, size_t>> ranges;
  size_t                                 n = strlen(msg);
  splitTopLevel(msg, n, ';', ranges);
  for(const auto &pr : ranges) {
    Command cmd;
    if(!parseSegment(msg + pr.first, pr.second - pr.first, cmd)) {
      return false;
    }
    out.push_back(cmd);
  }
  return true;
}

/**
 * @brief Extrai `T,C,B,j` de um comando header (`mode == 'h'`).
 * @param cmd Comando parseado.
 * @param[out] hdr Campos do header de lista.
 * @return `false` se `cmd` não for um header válido.
 */
inline bool parseListHeader(const Command &cmd, ListHeader &hdr) {
  if(cmd.mode != 'h' || cmd.argc < 6) {
    return false;
  }
  hdr.T = atoi(cmd.argv[2]);
  hdr.C = atoi(cmd.argv[3]);
  hdr.B = atoi(cmd.argv[4]);
  hdr.j = atoi(cmd.argv[5]);
  return true;
}

/**
 * @brief Vista conveniente dos argumentos de payload de um @ref Command.
 *
 * Ignora `modo`, `role` e (em bodies) o índice da linha.
 */
struct WireView {
  const Command &cmd; /**< Comando fonte. */

  /**
   * @brief Índice do primeiro argumento de payload em `cmd.argv`.
   * @return `3` para `mode == 'b'`, `2` caso contrário.
   */
  int payloadStart() const {
    if(cmd.mode == 'b') {
      return 3;
    }
    return 2;
  }

  /**
   * @brief Quantidade de argumentos de payload (exclui modo/role/índice).
   * @return Número de argumentos úteis do comando.
   */
  int payloadArgc() const {
    int start = payloadStart();
    return cmd.argc > start ? cmd.argc - start : 0;
  }

  /**
   * @brief Retorna o argumento de payload no índice `i`.
   * @param i Índice relativo ao payload (0 = primeiro argumento útil).
   * @return Ponteiro para o token ou `""` se fora do intervalo.
   */
  const char *arg(int i) const {
    int idx = payloadStart() + i;
    if(idx < 0 || idx >= cmd.argc) {
      return "";
    }
    return cmd.argv[idx];
  }

  /**
   * @brief Lê o argumento `i` como inteiro.
   * @param i Índice relativo ao payload.
   * @param[out] out Valor parseado.
   * @return `false` se o token for inválido.
   */
  bool asInt(int i, int &out) const { return parseInt(arg(i), out); }

  /**
   * @brief Lê o argumento `i` como ponto flutuante.
   * @param i Índice relativo ao payload.
   * @param[out] out Valor parseado.
   * @return `false` se o token for inválido.
   */
  bool asFloat(int i, float &out) const { return parseFloat(arg(i), out); }

  /**
   * @brief Copia o argumento `i` para `buf`.
   * @param i Índice relativo ao payload.
   * @param[out] buf Buffer de destino.
   * @param cap Capacidade de `buf`.
   * @return `false` se o valor não couber em `buf`.
   */
  bool asStr(int i, char *buf, size_t cap) const {
    const char *s = arg(i);
    if(strlen(s) >= cap) {
      return false;
    }
    memcpy(buf, s, strlen(s) + 1);
    return true;
  }
};

/**
 * @brief Normaliza o nome do comando para comparação case-insensitive.
 * @param name Nome do comando.
 * @return Cópia em minúsculas.
 */
inline std::string commandKeyLower(const char *name) {
  std::string k(name);
  for(char &c : k) {
    c = static_cast<char>(tolower(static_cast<unsigned char>(c)));
  }
  return k;
}

/**
 * @brief Serializa um body de lista `nome(b,role,idx,...);` em `buf`.
 * @param[out] buf Buffer de saída.
 * @param cap Capacidade de `buf`.
 * @param[in,out] pos Posição atual de escrita.
 * @param cmdName Nome do comando.
 * @param listMode Modo wire (`'b'` para body).
 * @param listRole `r` requisição ou `s` resposta.
 * @param idx Índice da linha na lista.
 * @param fields Argumentos da linha.
 * @return `false` se não couber em `buf`.
 */
inline bool appendListBody(char *buf, size_t cap, size_t &pos,
                           const char *cmdName, char listMode, char listRole,
                           int idx, const std::vector<const char *> &fields) {
  int nw = snprintf(buf + pos, cap - pos, "%s(%c,%c,%d", cmdName, listMode,
                    listRole, idx);
  if(nw < 0 || static_cast<size_t>(nw) >= cap - pos) {
    return false;
  }
  pos += static_cast<size_t>(nw);
  for(const char *f : fields) {
    if(!appendComma(buf, cap, pos)) {
      return false;
    }
    if(!appendWireToken(buf, cap, pos, f)) {
      return false;
    }
  }
  if(pos + 2 >= cap) {
    return false;
  }
  buf[pos++] = ')';
  buf[pos++] = ';';
  buf[pos]   = '\0';
  return true;
}

/**
 * @brief Serializa um header de lista `nome(h,role,T,C,B,j);` em `buf`.
 * @param[out] buf Buffer de saída.
 * @param cap Capacidade de `buf`.
 * @param[in,out] pos Posição atual de escrita.
 * @param cmdName Nome do comando.
 * @param role `r` requisição ou `s` resposta.
 * @param T Total de linhas.
 * @param C Linhas nesta mensagem.
 * @param B Total de mensagens.
 * @param j Índice desta mensagem.
 * @return `false` se não couber em `buf`.
 */
inline bool appendListHeader(char *buf, size_t cap, size_t &pos,
                             const char *cmdName, char role, int T, int C,
                             int B, int j) {
  char ts[16], cs[16], bs[16], js[16];
  snprintf(ts, sizeof(ts), "%d", T);
  snprintf(cs, sizeof(cs), "%d", C);
  snprintf(bs, sizeof(bs), "%d", B);
  snprintf(js, sizeof(js), "%d", j);
  int nw = snprintf(buf + pos, cap - pos, "%s(h,%c,%s,%s,%s,%s);", cmdName,
                    role, ts, cs, bs, js);
  if(nw < 0 || static_cast<size_t>(nw) >= cap - pos) {
    return false;
  }
  pos += static_cast<size_t>(nw);
  return true;
}

/**
 * @brief Calcula o tamanho em bytes de um header de lista serializado.
 * @param cmdName Nome do comando.
 * @param role `r` ou `s`.
 * @param T Total de linhas.
 * @param C Linhas nesta mensagem.
 * @param B Total de mensagens.
 * @param j Índice desta mensagem.
 * @return Comprimento UTF-8 do header (sem terminador nulo).
 */
inline size_t listHeaderWireBytes(const char *cmdName, char role, int T, int C,
                                  int B, int j) {
  char tmp[160];
  int  n = snprintf(tmp, sizeof(tmp), "%s(h,%c,%d,%d,%d,%d);", cmdName, role, T,
                    C, B, j);
  if(n < 0) {
    return sizeof(tmp);
  }
  return static_cast<size_t>(n);
}

/**
 * @brief Emissor de mensagens wire com callback injetável.
 *
 * @tparam MessageSize Tamanho máximo (bytes) de cada buffer de mensagem.
 *
 * A comunicação Bluetooth é responsabilidade do usuário da biblioteca;
 * `pushMessage` enfileira cada string pronta para envio.
 */
template <size_t MessageSize = 256> class Protocol {
public:
  /** @brief Callback que recebe cada mensagem wire completa para envio. */
  using PushMessageFn = void (*)(const char *msg);

  /**
   * @brief Constrói o emissor com a função de envio.
   * @param pushMessage Função chamada para cada mensagem montada.
   */
  explicit Protocol(PushMessageFn pushMessage) : pushMessage_(pushMessage) {}

  /**
   * @brief Emite um comando single `nome(s,role,...);`.
   * @param role `r` requisição ou `s` resposta.
   * @param cmdName Nome do comando.
   * @param parts Argumentos após modo e role.
   * @return `false` se a mensagem não couber em `MessageSize`.
   */
  bool emitSingle(char role, const char *cmdName,
                  const std::vector<const char *> &parts) const {
    char   buf[MessageSize];
    size_t pos = 0;
    int    nw  = snprintf(buf, sizeof(buf), "%s(s,%c", cmdName, role);
    if(nw < 0 || static_cast<size_t>(nw) >= sizeof(buf)) {
      return false;
    }
    pos = static_cast<size_t>(nw);
    for(const char *p : parts) {
      if(!appendComma(buf, sizeof(buf), pos)) {
        return false;
      }
      if(!appendWireToken(buf, sizeof(buf), pos, p)) {
        return false;
      }
    }
    if(pos + 2 >= sizeof(buf)) {
      return false;
    }
    buf[pos++] = ')';
    buf[pos++] = ';';
    buf[pos]   = '\0';
    push(buf);
    return true;
  }

  /**
   * @brief Emite `nome(s,r,...);` (requisição single).
   * @param cmdName Nome do comando.
   * @param parts Argumentos opcionais.
   * @return `false` se a mensagem não couber em `MessageSize`.
   */
  bool emitSingleRequest(const char                      *cmdName,
                         const std::vector<const char *> &parts = {}) const {
    return emitSingle('r', cmdName, parts);
  }

  /**
   * @brief Emite `nome(s,s,...);` (resposta single).
   * @param cmdName Nome do comando.
   * @param parts Argumentos da resposta.
   * @return `false` se a mensagem não couber em `MessageSize`.
   */
  bool emitSingleResponse(const char                      *cmdName,
                          const std::vector<const char *> &parts) const {
    return emitSingle('s', cmdName, parts);
  }

  /**
   * @brief Responde a uma requisição single recebida.
   * @param req Comando de entrada (`mode == 's'`, `role == 'r'`).
   * @param parts Argumentos da resposta.
   * @return `false` se `req` não for uma requisição single válida.
   */
  bool respondSingle(const Command                   &req,
                     const std::vector<const char *> &parts) const {
    if(req.role != 'r' || req.mode != 's') {
      return false;
    }
    return emitSingleResponse(req.name, parts);
  }

  /**
   * @brief Envia ACK de lote para coleta de listas no host.
   *
   * Formato multi-mensagem: `nome(s,s,0,B-1,ok);` após a última mensagem.
   * Mensagem única (B=1): `nome(s,s,0,0,ok);`
   *
   * @param cmdName Nome do comando da lista.
   * @param lo Índice inclusive da primeira mensagem confirmada.
   * @param hi Índice inclusive da última mensagem confirmada.
   */
  void emitBatchAckRange(const char *cmdName, int lo, int hi) const {
    char loBuf[16];
    char hiBuf[16];
    snprintf(loBuf, sizeof(loBuf), "%d", lo);
    snprintf(hiBuf, sizeof(hiBuf), "%d", hi);
    emitSingleResponse(cmdName, {loBuf, hiBuf, "ok"});
  }

  /**
   * @brief Envia ACK para uma única mensagem de lote (`lo == hi == j`).
   * @param cmdName Nome do comando da lista.
   * @param messageIndex Índice `j` da mensagem confirmada.
   */
  void emitBatchAck(const char *cmdName, int messageIndex) const {
    emitBatchAckRange(cmdName, messageIndex, messageIndex);
  }

  /**
   * @brief Formata um body de lista em `buf` sem enviar.
   * @param[out] buf Buffer de saída.
   * @param cap Capacidade de `buf`.
   * @param cmdName Nome do comando.
   * @param role `r` ou `s`.
   * @param idx Índice da linha.
   * @param fields Argumentos da linha.
   * @return `false` se não couber em `buf`.
   */
  bool formatListBody(char *buf, size_t cap, const char *cmdName, char role,
                      int idx, const std::vector<const char *> &fields) const {
    size_t pos = 0;
    return appendListBody(buf, cap, pos, cmdName, 'b', role, idx, fields);
  }

  /**
   * @brief Monta um segmento `nome(b,role,idx,...);` como `std::string`.
   * @param cmdName Nome do comando.
   * @param role `r` ou `s`.
   * @param idx Índice da linha.
   * @param fields Argumentos da linha.
   * @return String vazia em caso de falha (buffer insuficiente).
   */
  std::string
  makeListBodySegment(const char *cmdName, char role, int idx,
                      const std::vector<const char *> &fields) const {
    char   buf[MessageSize];
    size_t pos = 0;
    if(!appendListBody(buf, sizeof(buf), pos, cmdName, 'b', role, idx,
                       fields)) {
      return {};
    }
    return std::string(buf);
  }

  /**
   * @brief Empacota bodies já formatados em uma ou mais mensagens com headers.
   *
   * Cada mensagem gerada tem a forma `nome(h,role,T,C,B,j);body1;body2;...`
   * e é entregue via `pushMessage`. Usa @ref kPackBudget para fatiar listas
   * longas.
   *
   * @param cmdName Nome do comando.
   * @param role `r` ou `s`.
   * @param bodies Segmentos completos `nome(b,role,idx,...);`.
   */
  void emitListFromBodySegments(const char *cmdName, char role,
                                const std::vector<std::string> &bodies) const {
    int T = static_cast<int>(bodies.size());
    if(T == 0) {
      flushListChunk(cmdName, role, {}, 0, 1, 0);
      return;
    }
    const int B_pess = std::max(1, T);
    const int j_pess = std::max(0, T - 1);

    std::vector<std::vector<std::string>> chunks;
    std::vector<std::string>              cur;
    size_t                                sumBodies = 0;

    for(const std::string &b : bodies) {
      const int    nextC = static_cast<int>(cur.size()) + 1;
      const size_t hdrBytes =
          listHeaderWireBytes(cmdName, role, T, nextC, B_pess, j_pess);
      const size_t totalIfAdd = hdrBytes + sumBodies + b.size();
      if(!cur.empty() && totalIfAdd > kPackBudget) {
        chunks.push_back(std::move(cur));
        cur.clear();
        sumBodies = 0;
      }
      cur.push_back(b);
      sumBodies += b.size();
    }
    if(!cur.empty()) {
      chunks.push_back(std::move(cur));
    }

    int B = static_cast<int>(chunks.size());
    if(B < 1) {
      B = 1;
    }
    for(int j = 0; j < static_cast<int>(chunks.size()); j++) {
      flushListChunk(cmdName, role, chunks[static_cast<size_t>(j)], T, B, j);
    }
  }

  /**
   * @brief Emite uma lista de resposta (`role == 's'`).
   * @param cmdName Nome do comando.
   * @param bodies Segmentos `nome(b,s,idx,...);` já formatados.
   * @return Sempre `true` (erros de tamanho são silenciosos em `flushListChunk`).
   */
  bool emitListResponse(const char                     *cmdName,
                        const std::vector<std::string> &bodies) const {
    emitListFromBodySegments(cmdName, 's', bodies);
    return true;
  }

  /**
   * @brief Emite uma lista de requisição (`role == 'r'`).
   * @param cmdName Nome do comando.
   * @param bodies Segmentos `nome(b,r,idx,...);` já formatados.
   * @return Sempre `true`.
   */
  bool emitListRequest(const char                     *cmdName,
                       const std::vector<std::string> &bodies) const {
    emitListFromBodySegments(cmdName, 'r', bodies);
    return true;
  }

  /** @brief Uma linha de lista: vetor de argumentos de payload. */
  using ListRow = std::vector<const char *>;

  /**
   * @brief Monta bodies, empacota e envia a lista em uma única chamada.
   *
   * O índice de cada linha é atribuído automaticamente (0, 1, 2, …).
   * Para índices customizados (ex.: 1-based), inclua o índice em `rows[i]`.
   *
   * @param cmdName Nome do comando.
   * @param role `r` ou `s`.
   * @param rows Argumentos de cada linha (sem modo/role/índice wire).
   * @return `false` se alguma linha não couber em `MessageSize`.
   */
  bool emitListFromRows(const char *cmdName, char role,
                        const std::vector<ListRow> &rows) const {
    std::vector<std::string> bodies;
    bodies.reserve(rows.size());
    for(size_t i = 0; i < rows.size(); i++) {
      std::string seg =
          makeListBodySegment(cmdName, role, static_cast<int>(i), rows[i]);
      if(seg.empty() && !rows[i].empty()) {
        return false;
      }
      bodies.push_back(std::move(seg));
    }
    emitListFromBodySegments(cmdName, role, bodies);
    return true;
  }

  /**
   * @brief Atalho para @ref emitListFromRows com `role == 's'`.
   * @param cmdName Nome do comando.
   * @param rows Linhas da lista.
   * @return `false` se alguma linha não couber em `MessageSize`.
   */
  bool emitListResponseRows(const char                 *cmdName,
                            const std::vector<ListRow> &rows) const {
    return emitListFromRows(cmdName, 's', rows);
  }

  /**
   * @brief Atalho para @ref emitListFromRows com `role == 'r'`.
   * @param cmdName Nome do comando.
   * @param rows Linhas da lista.
   * @return `false` se alguma linha não couber em `MessageSize`.
   */
  bool emitListRequestRows(const char                 *cmdName,
                           const std::vector<ListRow> &rows) const {
    return emitListFromRows(cmdName, 'r', rows);
  }

  /**
   * @brief Emite uma lista sem reter cada body em `std::string`.
   *
   * Duas passagens: a primeira só mede o tamanho de cada body (buffer de
   * pilha) para saber `B`; a segunda formata e envia um pacote por vez.
   * O critério de corte é o mesmo de @ref emitListFromBodySegments
   * (header pessimista com `B == T`).
   *
   * @param cmdName Nome do comando.
   * @param role `r` ou `s`.
   * @param total Quantidade de linhas. Índices passados a `writeBody` são
   *        0-based.
   * @param writeBody `size_t(int index, char *buf, size_t cap)` grava um
   *        segmento `nome(b,role,idx,...);` e devolve o comprimento (sem o
   *        `NUL`), ou `0` se não couber.
   * @return `false` se algum body não puder ser formatado ou empacotado.
   */
  template <typename WriteBody>
  bool emitListStreaming(const char *cmdName, char role, int total,
                         WriteBody writeBody) const {
    if(total <= 0) {
      flushListChunk(cmdName, role, {}, 0, 1, 0);
      return true;
    }

    const int B_pess = std::max(1, total);
    const int j_pess = std::max(0, total - 1);
    char      body[MessageSize];

    auto bodyLenAt = [&](int index, size_t &outLen) -> bool {
      const size_t n = writeBody(index, body, sizeof(body));
      if(n == 0 || n >= sizeof(body)) {
        return false;
      }
      outLen = n;
      return true;
    };

    int    chunks    = 0;
    int    curCount  = 0;
    size_t sumBodies = 0;
    for(int i = 0; i < total; i++) {
      size_t len = 0;
      if(!bodyLenAt(i, len)) {
        return false;
      }
      const int    nextC = curCount + 1;
      const size_t hdrBytes =
          listHeaderWireBytes(cmdName, role, total, nextC, B_pess, j_pess);
      if(curCount > 0 && hdrBytes + sumBodies + len > kPackBudget) {
        chunks++;
        curCount  = 0;
        sumBodies = 0;
      }
      curCount++;
      sumBodies += len;
    }
    if(curCount > 0) {
      chunks++;
    }
    const int B = std::max(1, chunks);

    char   packed[MessageSize];
    size_t packedLen = 0;
    int    rowCount  = 0;
    int    j         = 0;

    auto flushPacked = [&]() -> bool {
      char   msg[MessageSize];
      size_t pos = 0;
      if(!appendListHeader(msg, sizeof(msg), pos, cmdName, role, total,
                           rowCount, B, j)) {
        return false;
      }
      if(pos + packedLen >= sizeof(msg)) {
        return false;
      }
      memcpy(msg + pos, packed, packedLen);
      pos += packedLen;
      msg[pos] = '\0';
      push(msg);
      return true;
    };

    for(int i = 0; i < total; i++) {
      size_t len = 0;
      if(!bodyLenAt(i, len)) {
        return false;
      }
      const int    nextC = rowCount + 1;
      const size_t hdrBytes =
          listHeaderWireBytes(cmdName, role, total, nextC, B_pess, j_pess);
      if(rowCount > 0 && hdrBytes + packedLen + len > kPackBudget) {
        if(!flushPacked()) {
          return false;
        }
        j++;
        rowCount  = 0;
        packedLen = 0;
      }
      if(packedLen + len >= sizeof(packed)) {
        return false;
      }
      memcpy(packed + packedLen, body, len);
      packedLen += len;
      rowCount++;
    }
    if(rowCount > 0 && !flushPacked()) {
      return false;
    }
    return true;
  }

  /**
   * @brief Responde a um header de lista recebido (`h,r`).
   * @param reqHeader Comando header de requisição.
   * @param bodies Segmentos de resposta já formatados.
   * @return `false` se `reqHeader` não for um header de requisição válido.
   */
  bool respondList(const Command                  &reqHeader,
                   const std::vector<std::string> &bodies) const {
    if(reqHeader.mode != 'h' || reqHeader.role != 'r') {
      return false;
    }
    return emitListResponse(reqHeader.name, bodies);
  }

private:
  PushMessageFn pushMessage_;

  /**
   * @brief Entrega uma linha wire ao callback de envio.
   * @param line Mensagem completa terminada em `;`.
   */
  void push(const char *line) const {
    if(pushMessage_ != nullptr) {
      pushMessage_(line);
    }
  }

  /**
   * @brief Monta e envia uma mensagem: header + bodies concatenados.
   * @param cmdName Nome do comando.
   * @param role `r` ou `s`.
   * @param bodySegments Bodies desta mensagem.
   * @param T Total de linhas da lista.
   * @param B Total de mensagens.
   * @param j Índice desta mensagem.
   */
  void flushListChunk(const char *cmdName, char role,
                      const std::vector<std::string> &bodySegments, int T,
                      int B, int j) const {
    char   buf[MessageSize];
    size_t pos = 0;
    int    C   = static_cast<int>(bodySegments.size());
    if(!appendListHeader(buf, sizeof(buf), pos, cmdName, role, T, C, B, j)) {
      return;
    }
    for(const std::string &seg : bodySegments) {
      size_t len = seg.size();
      if(pos + len + 1 >= sizeof(buf)) {
        return;
      }
      memcpy(buf + pos, seg.c_str(), len);
      pos += len;
    }
    buf[pos] = '\0';
    push(buf);
  }
};

} // namespace wire

#endif // WPROTOCOL_HPP
