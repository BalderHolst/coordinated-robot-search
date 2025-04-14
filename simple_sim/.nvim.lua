-- run ```:luafile .nvim.lua``` after startup to load this file
-- This uses https://github.com/mrcjkb/rustaceanvim
-- Normally lsp-config is used.

local config = {
	cargo = {
		allFeatures = false,
		features = { "default" },
	},
}

local config_str = vim.inspect(config)

vim.cmd.RustAnalyzer({ "config", config_str })
vim.cmd.RustAnalyzer({ "restart" })
